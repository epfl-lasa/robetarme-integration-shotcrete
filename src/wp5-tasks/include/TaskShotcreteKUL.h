#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>
#include <string>

#include "ITaskBase.h"

class TaskShotcreteKUL : public ITaskBase {
public:
  TaskShotcreteKUL(double freq, std::string robotName);

  bool computePath();
  bool execute();
  bool humanSharing();
  void publishTwistPoseKUL(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> stateJoints);

private:
};
