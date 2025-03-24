#include "TaskShotcreteKUL.h"

#include <sys/select.h>

#include <cmath> // For std::round

using namespace std;
using namespace Eigen;

TaskShotcreteKUL::TaskShotcreteKUL(double freq, string robotName) : ITaskBase(freq, robotName) {
  // ros::NodeHandle nodeHandle = getRosNodehandle_();

  // Create an unique pointer for the instance of Tool
  // tools_ = make_unique<ToolsShotcrete>(nodeHandle);
  // while (rclcpp::ok() && tools_->getState()) {
  //   rosInterface_->spinNode();
  //   loopRate_->sleep();
  //   tools_->activateTool(false);
  //   cout << "waiting for the tool to be activated" << endl;
  // }
}

bool TaskShotcreteKUL::computePath() {
  bool success = false;
  vector<double> FirstQuatPos(7, 0.0);

  cout << "waiting for the first position to be received" << endl;
  tuple<vector<double>, vector<double>, vector<double>> stateJoints;
  rosInterface_->spinNode();
  stateJoints = rosInterface_->receiveState();
  vector<double> initialJoint = get<0>(stateJoints);

  while (!rosInterface_->readyToWorkingPosition() && rclcpp::ok()) {
    rosInterface_->spinNode();
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    vector<double> desiredJointSpeed(actualJoint.size());
    for (size_t i = 0; i < actualJoint.size(); ++i) {
      desiredJointSpeed[i] = (initialJoint[i] - actualJoint[i]);
    }
    rosInterface_->sendState(desiredJointSpeed);
    loopRate_->sleep();
  }

  FirstQuatPos = rosInterface_->getWorkingPose();
  stateJoints = rosInterface_->receiveState();
  vector<double> actualJoint = get<0>(stateJoints);

  pair<int, vector<double>> ikResult = roboticArm_->getTracIK(actualJoint, FirstQuatPos);
  if (FirstQuatPos != vector<double>(7, 0.0) && ikResult.first > 0) {
    success = true;
    startJoint_ = ikResult.second;
  }
  return success;
}

void TaskShotcreteKUL::publishTwistPoseKUL(tuple<vector<double>, vector<double>, vector<double>> stateJoints) {
  vector<double> actualJoint = get<0>(stateJoints);
  vector<double> actualJointSpeed = get<1>(stateJoints);

  pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFKPinocchio(actualJoint);
  VectorXd actualTwist = roboticArm_->getTwistFromJointState(actualJoint, actualJointSpeed);

  rosInterface_->setActualCartesianPoseEEF(pairActualQuatPos);
  rosInterface_->setActualCartesianTwistEEF(actualTwist);
}

bool TaskShotcreteKUL::execute() {
  bool success = false;
  std::cout << "waiting for the shotcrete to begin..." << endl;
  // TODO: add rosservice to start shotcrete
  tuple<vector<double>, vector<double>, vector<double>> stateJoints;
  rosInterface_->spinNode();
  stateJoints = rosInterface_->receiveState();
  vector<double> initialJoint = get<0>(stateJoints);

  while (rclcpp::ok() && !rosInterface_->isShotcreteReadyToBegin()) {
    rosInterface_->spinNode();
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    vector<double> desiredJointSpeed(actualJoint.size());
    for (size_t i = 0; i < actualJoint.size(); ++i) {
      desiredJointSpeed[i] = (initialJoint[i] - actualJoint[i]);
    }
    rosInterface_->sendState(desiredJointSpeed);
    loopRate_->sleep();
    // publish pose and twist actual
    publishTwistPoseKUL(stateJoints);
  }

  cout << "Let's shotcrete begin..." << endl;

  while (rclcpp::ok() && !rosInterface_->isShotcreteDone()) {
    tuple<vector<double>, vector<double>, vector<double>> stateJoints = rosInterface_->receiveState();
    Eigen::VectorXd twistKUL = rosInterface_->getTwistKUL();
    vector<double> desiredJointSpeed = roboticArm_->lowLevelController(stateJoints, twistKUL);
    rosInterface_->sendState(desiredJointSpeed);
    //publishe pose and twist actual
    publishTwistPoseKUL(stateJoints);
    rosInterface_->spinNode();
    loopRate_->sleep();
  }
  cout << "Shotcrete done go to home..." << endl;

  return rosInterface_->isShotcreteDone();
}

bool TaskShotcreteKUL::humanSharing() {
  cout << "Not implemented" << endl;
  return false;
}
