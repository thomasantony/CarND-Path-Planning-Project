//
// Created by Thomas Antony on 8/26/17.
//

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "fsm.h"
enum class States { KL, LCL, LCR };
enum class Triggers { StayInLane, DoLaneChangeLeft, DoLaneChangeRight };

// Models car behavior using a finite state machine
namespace tantony {
  class Behavior {
  private:
    FSM::Fsm<States, States::KL, Triggers> fsm;
    int target_lane_;
    double target_speed_;
  public:
    Behavior(): target_lane_(1), target_speed_(49.5*.447) {}
    Trajectory& GetTrajectory(const Car& ego,
                              const Map& map,
                              const Trajectory& prev_path){

    }
  };
}
#endif //BEHAVIOR_H
