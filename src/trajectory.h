#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
/**
  Trajectory generation module
*/

using dvector = std::vector<double>;

namespace tantony {
  class Trajectory {
  private:
    dvector map_s_, map_x_, map_y_;
    double anchor_spacing_;
    double lane_width_;
  public:
    Trajectory(dvector& map_s, dvector &map_x, dvector& map_y, double anchor_spacing, double lane_width ) :
      map_s_(map_s),
      map_x_(map_x),
      map_y_(map_y),
      anchor_spacing_(anchor_spacing),
      lane_width_(lane_width)
      {}
    void generate(int target_lane, double target_speed);
  };
}

#endif // TRAJECTORY_H
