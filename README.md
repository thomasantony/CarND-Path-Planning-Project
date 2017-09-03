# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Implementation Details

### Overview

The project consists of the following files:

 - `types.h` - Defines all the custom data structures used by the program
 - `behavior.h` - Contains the `Behavior` class that uses a finite state machine for behavior planning. It generates a target speed and a target lane for the ego car.
 - `trajectory.h` - Uses a commanded lane and target speed to generate a smooth trajectory that does not exceed the limits in acceleration, jerk etc.

### Trajectory generation

The trajectory generator uses a spline to generate smooth paths for the car to follow. The
trajectory is computed by the `GenerateTrajectory` function in `trajectory.h`. A line by line description of the function follows:

```
const Trajectory GenerateTrajectory(const Command& cmd,
                                    const Car& ego,
                                    const Map& map,
                                    const Trajectory& prev_path)
{
```
The function takes the following arguments:
 - A `Command` object that provides contains a target lane and a speed
 - A `Car` object that describes the state of the ego car
 - A `Map` object that contains the predefined map waypoints (in XY and Frenet coordinates)
 - A `Trajectory` object containing the path computed in the previous iteration as a list of (x, y) coordinates.

```
vector<double> spline_anchor_x, spline_anchor_y;

const auto num_prev_points = prev_path.size();
// Reference state for coordinate transformation
// Start with car state as reference
double ref_x = ego.x;
double ref_y = ego.y;
double ref_yaw = ego.yaw;

double ego_s = ego.s;
if (num_prev_points > 2) {
  // Else start at the end of prior waypoints
  // and use them as reference for coordinate transform
  ref_x = prev_path.x[num_prev_points-1];
  ref_y = prev_path.y[num_prev_points-1];

  // Estimate past heading
  double prev_wp_x = prev_path.x[num_prev_points-2];
  double prev_wp_y = prev_path.y[num_prev_points-2];
  ref_yaw = atan2(ref_y - prev_wp_y,
                  ref_x - prev_wp_x);

  spline_anchor_x.push_back(prev_path.x[num_prev_points-2]);
  spline_anchor_y.push_back(prev_path.y[num_prev_points-2]);

  ego_s = prev_path.end_path_s; // Helpful for making smooth trajectories
}
// position at t
spline_anchor_x.push_back(ref_x);
spline_anchor_y.push_back(ref_y);
```
Here we first define the "reference states" used to convert the global Cartesian coordinates obtained from the simulator into local coordinates. The last point of the previous trajectory is used as a reference if possible. If there is no previous path available, the state of the ego car is used as the reference.

`spline_anchor_x` and `spline_anchor_y` are "anchor points" used to define the spline. If there is a prior path available, the final two points in the prior path is used as the first two anchor points for the spline.

This line: `ego_s = prev_path.end_path_s; // Helpful for making smooth trajectories` ensures that the spline starts from the previously computed path, ensuring a smooth transition.

```
// Add equally spaced points in Frenet space
for(int i = 0; i < 3; i++)
{
  auto wp = getXY(ego_s + anchor_spacing*(i+1),
                  (2+4*cmd.lane),
                  map);
  spline_anchor_x.push_back(wp[0]);
  spline_anchor_y.push_back(wp[1]);
}
```
Three more anchor points, are generated as equally spaced points in Frenet space in the target lane in front of the ego car.

```
// Convert spline points to local coordinates
auto ref_state = std::make_tuple(ref_x, ref_y, ref_yaw);
for(int i=0; i<spline_anchor_x.size(); i++)
{
  auto x = spline_anchor_x[i];
  auto y = spline_anchor_y[i];
  std::tie(x, y) = GlobalToLocal(std::make_tuple(x, y), ref_state);
  spline_anchor_x[i] = x;
  spline_anchor_y[i] = y;
}
tk::spline s;
// Fit spline to anchor points
s.set_points(spline_anchor_x, spline_anchor_y);

```
The anchor points are converted into local coordinates for numerical stability and spline is generated using the `tk::spline` class.

```
// Copy old waypoints first
Trajectory output;

for(int i = 0; i < num_prev_points; i++)
{
  output.x.push_back(prev_path.x[i]);
  output.y.push_back(prev_path.y[i]);
  auto sd = getFrenet(prev_path.x[i], prev_path.y[i], ego.yaw, map);
  output.s.push_back(sd[0]);
  output.d.push_back(sd[1]);
}
```
The waypoints from the previous iteration as used as the start for the new path. The path planner only "fills in" as many points as were traversed by the car after the past iteration. For example, if the original path consisted of 150 points and the car has traversed through 25 of them, the new path will be the 125 "old" waypoints along with 25 new waypoints. This is another factor that helps ensure smooth paths that do not violate constraints.

```
// Find waypoint spacing based on speed
const auto target_x = anchor_spacing;
const auto target_y = s(target_x);
const auto target_dist = sqrt(target_x*target_x + target_y*target_y);
double N = target_dist / (timestep*cmd.speed);
```
The ideal spacing for new waypoints is computed based on the commanded speed.

```
// Find new points and transform back to global space
double last_x = 0;
for(int i = 1; i <= 50 - num_prev_points; i++)
{
  // double wp_x = last_x + anchor_spacing/num_points;
  double wp_x = last_x + target_x/N;
  double wp_y = s(wp_x);

  last_x = wp_x;
  tie(wp_x, wp_y) = LocalToGlobal(make_tuple(wp_x, wp_y), ref_state);

  output.x.push_back(wp_x);
  output.y.push_back(wp_y);
}
return output;
```
New waypoints are computed using this spacing and converted back to global coordinates for sending back to the simulator.

### Behavior Planning

The behavior planner uses relatively simple logic to decide a safe lane and speed for the ego car. It is implemented using a Finite State Machine mainly to allow for easy expansion in the future. The behavior planning logic is implemented in the `UpdateState` method of the `Behavior` class. This method is called by `main.cpp` in each iteration. It uses ego car telemetry and sensor fusion data on traffic to determine the correct course of action. The three states that are currently implemented are "Keep Lane" (KL), "Lane Change Left" (LCL), and "Lane Change Right" (LCR).

The method mainly consists of an if-else block that performs different actions based on the current state of the car.

```
if (current_state == States::KL) {
  // Find closest car in front
  double min_s = 9999.0;
  bool will_collide = false;
  for (auto &car : traffic) {
    double collision_horizon = prev_path.size() * timestep;
    will_collide = CheckFrontCollision(ego, car, prev_path,  ego.lane, collision_horizon);
    if (will_collide) {
      break;
    }
  }
```
This code block describes the behavior when in the KL state. First, we check for any possible front collisions and set the `will_collide` flag. The collision checks are implemented in the `CheckFrontCollision` function.

```
if (will_collide) {
  const bool do_lcl = false, do_lcr = false;
  const LaneInfo& left_lane = CheckLane(ego, traffic, ego.lane - 1);
  const LaneInfo& right_lane = CheckLane(ego, traffic, ego.lane + 1);

  // If both lanes are clear, pick one with bigger buffer
  if(left_lane.clear && right_lane.clear)
  {
    if (left_lane.buffer >= right_lane.buffer) {
      fsm_.execute(Triggers::DoLaneChangeLeft);
    } else {
      fsm_.execute(Triggers::DoLaneChangeRight);
    }
  } else if (left_lane.clear) {
    fsm_.execute(Triggers::DoLaneChangeLeft);
  } else if (right_lane.clear) {
    fsm_.execute(Triggers::DoLaneChangeRight);
  }
}
```
If `will_collide` is set, we then check the left and right lanes. If both lanes are clear, we select the one with a bigger "buffer" distance to the closest car. If only one lane is clear, we select that lane for a lane change.

```
else if (current_state == States::LCL)
{
  auto* lc_state = dynamic_cast<LaneChangeState*>(state_);
    // Check if ego car in the targeted lane
    in_correct_lane = ego.d > (2 + (4*lc_state->target_lane_-2)) && (ego.d < (2+4*lc_state->target_lane_+2));
    if(in_correct_lane)
    {
      // cout<<"Switching state to KL from LCL"<<endl;
      fsm_.execute(Triggers::StayInLane);
    }
}else if(current_state == States::LCR)
{
  auto* lc_state = dynamic_cast<LaneChangeState*>(state_);
    // Check if ego car in the targeted lane
    in_correct_lane = ego.d > (2 + (4*lc_state->target_lane_-2)) && (ego.d < (2+4*lc_state->target_lane_+2));
    if(in_correct_lane)
    {
      // cout<<"Switching state to KL from LCR"<<endl;
      fsm_.execute(Triggers::StayInLane);
    }
}
```
When in the lance change state, we wait until we are in the target lane before switching back to the "Keep Lane" state.

```
// Realize current state
Command cmd;
state_->RealizeState(ego, traffic, map, prev_path, cmd);
current_speed_ = cmd.speed;
current_lane_  = cmd.lane;

return GenerateTrajectory(cmd, ego, map, prev_path);
```

Here we use the current state of the FSM to generate a commanded speed and lane to be passed into the trajectory generator.


```
void RealizeStayInLane(
    const Car& ego,
    const vector<Car>& traffic,
    const Map& map,
    const Trajectory& prev_path,
    Command& cmd)
  {

  double target_speed = speed_limit;
  double closest_approach = collision_check_margin;
  // Loop over other detected cars
  // Match speed of closest car in front
  for(int i=0; i<traffic.size(); i++)
  {
    const Car& car = traffic[i];
    double collision_horizon = prev_path.size()*timestep;
    if(CheckFrontCollision(ego, car, prev_path, ego.lane, collision_horizon))
    {
      // Use proportional controller to maintain buffer
      auto dist_to_car = car.s - ego.s;
      if(dist_to_car < closest_approach)
      {
        target_speed = min(car.speed, target_speed);
        closest_approach = dist_to_car;
      }
    }
  }
```
This is the function that computes the commanded lane and speed for the "Keep Lane" state. The target speed is initially set to the speed limit. The function then loops through all the other cars which are in the same lane as the ego car and finds the one that is the closest (which is also within the "collision margin"). The speed of this car (if one exists) is selected as the target speed for safety.

```
// Adjust for maintaining buffer
static const auto gain = 5.0;
auto extra_speed = gain*(closest_approach - buffer_distance);
target_speed = max(0.5*.447, min(target_speed+extra_speed, speed_limit));
```
Here, we use a proportional gain controller to maintain a safe distance to the car in front.

```
// Gentle acceleration
double delta_speed = target_speed - ego.speed;
delta_speed = std::min(max_acceleration*timestep, std::max(-max_deceleration*timestep, delta_speed));
double cmd_speed = ego.speed + delta_speed;

cmd = {ego.lane, cmd_speed};
```
The commanded speed is slowly increased in accordance with the acceleration limits.

```
virtual void RealizeState(const Car& ego,
                  const vector<Car>& traffic,
                  const Map& map,
                  const Trajectory& prev_path,
                  Command& cmd) {

  const LaneInfo& laneinfo = CheckLane(ego, traffic, target_lane_);
  if(laneinfo.clear)
  {
    // Perform lane change
    int cmd_lane = min(lanes_available - 1, max(0, target_lane_));

    // Use "Stay in lane" logic to compute safe speed for target lane
    Car fake_ego(ego);
    fake_ego.lane = cmd_lane;
    RealizeStayInLane(fake_ego, traffic, map, prev_path, cmd);
  }else{
    // cout<<"Lane not clear ..."<<endl;
    RealizeStayInLane(ego, traffic, map, prev_path, cmd);
  }
}
```
The `RealizeState` method in the `LaneChangeState` class describes how a lane change is performed. First we check if the target lane is clear, if it is not, we stay in the current lane. If the target lane is clear, we use the `StayInLane` logic to compute a safe target speed in the new lane.

---

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!
