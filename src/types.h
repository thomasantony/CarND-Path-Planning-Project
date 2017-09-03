//
// Created by Thomas Antony on 8/26/17.
//

#ifndef PATH_PLANNING_TYPES_H
#define PATH_PLANNING_TYPES_H

struct Car {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  double speed;
  double yaw = 0.0;
  int lane = 0;
  double s_actual; //
};
typedef struct Car Car;
// Used by JSON module to parse JSON into Car object
void from_json(const json& j, Car& c) {
  c.id = j[0];
  c.x = j[1];
  c.y = j[2];
  c.vx = j[3];
  c.vy = j[4];
  c.speed = sqrt(c.vx*c.vx + c.vy*c.vy);
  c.s = j[5];
  c.d = j[6];
}

struct Map {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
};
typedef struct Map Map;
struct Trajectory {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> d;
  double end_path_s = 0.0;
  double end_path_d = 0.0;

  inline size_t size() const{
    return x.size();
  }
};
typedef struct Trajectory Trajectory;
struct Command {
  int lane;
  double speed;
};
typedef struct Command Command;

struct LaneInfo{
  int number;
  bool clear;
  double speed;
  double buffer;
};
using LaneInfo = struct LaneInfo;

#endif //PATH_PLANNING_TYPES_H
