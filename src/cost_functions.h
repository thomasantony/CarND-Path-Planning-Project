//
// Created by Thomas Antony on 8/26/17.
//

#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H

#include <functional>

// priority levels for costs
static const auto COLLISION  = 1E+6;
static const auto DANGER     = 1E+5;
static const auto COMFORT    = 1E+3;
static const auto EFFICIENCY = 1E+2;

static const auto DESIRED_BUFFER = 1.5; // timesteps
static const auto PLANNING_HORIZON = 2;


struct TrajectoryData {
  double avg_speed;
  double closest_approach;
  bool collides;
};
using TrajectoryData = struct TrajectoryData;

//bool CheckCollision()
//using CostFunction = std::function<double(const Trajectory&, const Car&, const vector<Car>&, const Map&)>;
//const TrajectoryData ComputeTrajectoryData(const Trajectory& trajectory,
//                                           const Car& ego,
//                                           const vector<Car>& traffic,
//                                           const Map& map)
//{
//  TrajectoryData out;
//
//  auto t = trajectory;
//  auto current_snapshot = t[0];
//  auto first = t[1];
//  auto last = myvector.back();
//
//  // Might need to fix wraparound
//  auto dt = t.size() * timestep;
//  auto closest_approach = 999999.0;
//
//  out.avg_speed = (last.s - current_snapshot.s) / dt;
//  out.collides = false;
//
//  // Loop through traffic
//  for(auto& car : traffic)
//  {
//    if(abs(car.d - ego.d) < 0.1)
//    {
//
//    }
//  }
//
//  out.closest_approach = 0.0;
//}
double BufferCost(const TrajectoryData& data, const Car& car, const vector<Car>& traffic, const Map& map)
{
  return 0.0;
}

double InefficiencyCost(const TrajectoryData& data, const Car& car, const vector<Car>& traffic, const Map& map)
{
  // Try to drive at the speed limit all the time
  auto delta = data.avg_speed - speed_limit;
  return abs(delta)/speed_limit * EFFICIENCY;
}
double CollisionCost(const TrajectoryData& data, const Car& car, const vector<Car>& traffic, const Map& map)
{
  if (data.collides)
  {
    return 1.0*COLLISION;
  }
  return 0.0;
}

auto CostFunctionList = {InefficiencyCost, CollisionCost};

double CalculateCost(const Trajectory& trajectory,
                     const Car& ego,
                     const vector<Car>& traffic,
                     const Map& map){
  double total = 0.0;
//  TrajectoryData data = ComputeTrajectoryData(trajectory, ego, traffic, map);
  return total;
}
#endif //COST_FUNCTIONS_H
