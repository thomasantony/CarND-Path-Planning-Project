//
// Created by Thomas Antony on 8/26/17.
//

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

using namespace std;

#include "types.h"
// Finite State Machine
#include "fsm.h"
// Trajectory generator
#include "trajectory.h"

// Checks if car will collide with ego car, `delta_t` seconds in future
const bool CheckFrontCollision(const Car& ego, const Car& car, const Trajectory& prev_path, int lane, const double delta_t)
{
  double future_ego_s;
  if(prev_path.size() > 0)
  {
    future_ego_s = prev_path.end_path_s; // Use the "right" s value for collision prediction
  }else{
    future_ego_s = ego.s;
  }

  const auto future_s = car.s + car.speed*delta_t;
  future_ego_s += ego.speed*delta_t;
  const auto in_same_lane = (car.d > (2 + (4*lane-2)) && (car.d < (2+4*lane+2)));

  if (in_same_lane)
  {
    const auto in_front = (future_s >= future_ego_s-2.5); // Account for length of car
    const auto too_close= ((future_s - future_ego_s-2.5) < collision_check_margin);
    return in_front && too_close;
  }
  return false;
}

const LaneInfo CheckLane(const Car& ego, const vector<Car>& traffic, int lane)
{
  // Upper and lower bounds of region to check for cars
  const auto s_ub = ego.s + lane_check_front_margin;
  const auto s_lb = ego.s + lane_check_back_margin;

  // Upper and lower bounds of region to check for cars in future
  const auto future_s_ub = ego.s + lane_check_front_margin + ego.speed*lane_check_horizon;
  const auto future_s_lb = ego.s + lane_check_back_margin + ego.speed*lane_check_horizon;

  double lane_buffer = 999.0;

  if(lane < 0 || lane >= lanes_available)
  {
    return {false, 0.0};
  }
  for (auto& car: traffic)
  {
    const auto future_s = car.s + car.speed*lane_check_horizon;
    const auto in_lane = (car.d > (2 + (4*lane-2)) && (car.d < (2+4*lane+2)));
    if (in_lane)
    {
      const auto in_danger_zone = (s_lb < car.s) && (car.s < s_ub);
      const auto future_danger = (future_s_lb < future_s) && (future_s < future_s_ub);
      if (in_danger_zone or future_danger)
      {
        return {false, 0.0};
      }else{
        // Lane is clear, find speed limit and buffer
        auto buffer = car.s - ego.s;
        if(buffer < lane_buffer)
        {
          lane_buffer = buffer;
        }
      }
    }
  }
  return {true, lane_buffer};
}
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

  // Adjust for maintaining buffer
  static const auto gain = 5.0;
  auto extra_speed = gain*(closest_approach - buffer_distance);
  target_speed = max(0.5*.447, min(target_speed+extra_speed, speed_limit));

  // Gentle acceleration
  double delta_speed = target_speed - ego.speed;
  delta_speed = std::min(max_acceleration*timestep, std::max(-max_deceleration*timestep, delta_speed));
  double cmd_speed = ego.speed + delta_speed;

  cmd = {ego.lane, cmd_speed};
}

class VehicleState {
public:
  VehicleState() {}
  virtual void RealizeState(const Car& ego,
                            const vector<Car>& traffic,
                            const Map& map,
                            const Trajectory& prev_path,
                            Command& cmd) = 0;
  virtual ~VehicleState() {}
};

class KeepLaneState : public VehicleState {
public:
  virtual void RealizeState(const Car& ego,
                    const vector<Car>& traffic,
                    const Map& map,
                    const Trajectory& prev_path,
                    Command& cmd) {
    RealizeStayInLane(ego, traffic, map, prev_path, cmd);
  }
};
class LaneChangeState : public VehicleState {
public:
  int target_lane_;
  LaneChangeState(int lane): target_lane_(lane) {}
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
};
enum class States { KL=0, LCL, LCR };
enum class Triggers { StayInLane, DoLaneChangeLeft, DoLaneChangeRight };
// Helper function for printing state
// Source: https://stackoverflow.com/a/3342891/538379
std::ostream& operator<<(std::ostream& out, const States value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
    switch(value){
        PROCESS_VAL(States::KL);
        PROCESS_VAL(States::LCL);
        PROCESS_VAL(States::LCR);
    }
#undef PROCESS_VAL

    return out << s;
}

class Behavior {
private:
  FSM::Fsm<States, States::KL, Triggers> fsm_;
  int current_lane_;
  double current_speed_;
  Command target_;
  VehicleState* state_;
public:
  Behavior(): current_lane_(1), current_speed_(0.0) {
    state_ = new KeepLaneState();
    fsm_.add_transitions({
             { States::KL, States::LCL, Triggers::DoLaneChangeLeft,[&]{return true;}, [&]{
               delete state_;
               state_ = new LaneChangeState(min(max(0, current_lane_ - 1), lanes_available - 1));
             } },
             { States::KL, States::LCR, Triggers::DoLaneChangeRight,[&]{return true;}, [&]{
               delete state_;
               state_ = new LaneChangeState(min(max(0, current_lane_ + 1), lanes_available - 1));
             } },
             { States::LCL, States::KL, Triggers::StayInLane,[&]{return true;}, [&]{delete state_; state_ = new KeepLaneState();} },
             { States::LCR, States::KL, Triggers::StayInLane,[&]{return true;}, [&]{delete state_; state_ = new KeepLaneState();} },
    });
  }

  const Trajectory UpdateState(Car ego,
                               const vector<Car> &traffic,
                               const Map &map,
                               const Trajectory &prev_path){
    // Returns trajectory for current state of FSM
    double collision_horizon = prev_path.size()*timestep;
    double target_speed;
    int target_lane;

    ego.lane = current_lane_;
    ego.speed = current_speed_;

    bool in_correct_lane;
    auto current_state = fsm_.state();
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
      cout<<fsm_.state()<<" ";
      cout<<"Front:"<<(will_collide?"Blocked":"Clear")<<" ";

      if (will_collide) {
        const bool do_lcl = false, do_lcr = false;
        const LaneInfo& left_lane = CheckLane(ego, traffic, ego.lane - 1);
        const LaneInfo& right_lane = CheckLane(ego, traffic, ego.lane + 1);

        cout<<"Left:"<<(left_lane.clear?"Clear":"Blocked")<<" ";
        cout<<"Right:"<<(left_lane.clear?"Clear":"Blocked")<<"\n";

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
        // Stay in lane if lane change not possible
      }else{
        cout<<endl;
      }
    }
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
    }else{
      cout<<"Invalid state!"<<endl;
    }

    // Realize current state
    Command cmd;
    state_->RealizeState(ego, traffic, map, prev_path, cmd);
    current_speed_ = cmd.speed;
    current_lane_  = cmd.lane;
    // Store computed trajectory for staying in lane
    return GenerateTrajectory(cmd, ego, map, prev_path);
  }
};
#endif //BEHAVIOR_H
