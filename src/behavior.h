//
// Created by Thomas Antony on 8/26/17.
//

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "fsm.h"
enum class States { KL, LCL, LCR };
enum class Triggers { StayInLane, DoLaneChangeLeft, DoLaneChangeRight };

static auto RealizeStayInLane = [](
    const Car& ego,
    const vector<Car>& traffic,
    const Map& map,
    const Trajectory& prev_path,
    Trajectory& output){
  double target_speed = speed_limit;

  // Do not change lane
  int target_lane = ego.lane;

  // Loop over other detected cars
  for(int i=0; i<traffic.size(); i++)
  {
    const Car& car = traffic[i];
    double collision_horizon = prev_path.size()*timestep;
    if(CheckCollision(ego, car, ego.lane, collision_horizon))
    {
      target_speed = min(car.speed, speed_limit);
      break;
    }
  }
  // Store computed trajectory for staying in lane
  output = GenerateTrajectory(target_lane, target_speed, ego, map, prev_path);
};
static auto RealizeLaneChangeLeft = [](
    const Car& ego,
    const vector<Car>& traffic,
    const Map& map,
    const Trajectory& prev_path,
    Trajectory& output){};

static auto RealizeLaneChangeRight = [](
    const Car& ego,
    const vector<Car>& traffic,
    const Map& map,
    const Trajectory& prev_path,
    Trajectory& output){};

using TrajectoryGenerator = std::function<void(const Car& ego,
                                                const vector<Car>& traffic,
                                                const Map& map,
                                                const Trajectory& prev_path,
                                                    Trajectory& output)>;

static std::map<States, TrajectoryGenerator> TrajGenFunctions = {{States::KL, RealizeStayInLane},
                                                                 {States::LCL, RealizeLaneChangeLeft},
                                                                 {States::LCR, RealizeLaneChangeRight}};
class Behavior {
private:
  FSM::Fsm<States, States::KL, Triggers> fsm_;
  int current_lane_;
  double current_speed_;
  double speed_limit_;
public:
  Behavior(): current_lane_(1), current_speed_(0.0), speed_limit_(49.5*0.447) {}
  const Trajectory UpdateState(Car ego,
                               const vector<Car> &traffic,
                               const Map &map,
                               const Trajectory &prev_path){
    // Returns trajectory for current state of FSM
    double collision_horizon = prev_path.size()*timestep;
    double target_speed;
    int target_lane;


    vector<States> available_states = {States::KL};
    if(ego.lane > 0)
    {
      available_states.push_back(States::LCL);
//      available_states.push_back("PLCL");
    }
    if(ego.lane < lanes_available-1)
    {
      available_states.push_back(States::LCR);
//      available_states.push_back("PLCR");
    }
    // Find cost for each possible state
    vector<double> costs(available_states.size());
    for(auto& state : available_states)
    {
      Trajectory pred_traj;
      auto traj_gen = TrajGenFunctions[state];
      traj_gen(ego, traffic, map, prev_path, pred_traj);
      // auto cost = find_cost(pred_traj);
      costs.push_back(0.0);
    }

    ego.lane = current_lane_;

    if(fsm_.state() == States::KL)
    {
      target_speed = speed_limit_;
      // Loop over other detected cars
      for(int i=0; i<traffic.size(); i++)
      {
        const Car& car = traffic[i];
        if(CheckCollision(ego, car, current_lane_, collision_horizon))
        {
          target_speed = min(car.speed, speed_limit_);
          break;
        }
      }
    }

    return GenerateTrajectory(target_lane, target_speed, ego, map, prev_path);
  }
};
#endif //BEHAVIOR_H
