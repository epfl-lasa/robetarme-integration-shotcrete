/**
 * @file RoboticArmCobod.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

// clang off
#include "IRoboticArmBase.h"
// clang on

#include <Eigen/Dense>
#include <memory>
#include <vector>

/**
 * @brief Child class to create all the prototype fonctions needed in the
 * different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary
 * functions to control it.
 */
class RoboticArmCobod : public IRoboticArmBase {
public:
  explicit RoboticArmCobod();
  // ~RoboticArmCobod();

  std::vector<double> lowLevelController(
      const std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& stateJoints,
      const Eigen::VectorXd& twist) override;
  std::vector<double> lowLevelControllerSF(
      const std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& stateJoints,
      const Eigen::VectorXd& desiredTwist,
      const Eigen::VectorXd& wrenchFromSensor) override;

  // /**
  //  * @brief Get the forward kinematics of the robotic arm.
  //  * @param jointPos Joint positions of the robotic arm.
  //  */
  // std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFKGeo(const std::vector<double>& jointPos) override;

  // /**
  //  * @brief Get the inverse kinematics of the robotic arm.
  //  * @param quaternion Quaternion of the end effector.
  //  * @param position Position of the end effector.
  //  * @param jointPos Vector of joint positions of the robotic arm.
  //  * @return Pair of the return code and the next joint positions.
  //  */
  // bool getIKGeo(const Eigen::Quaterniond& quaternion,
                // const Eigen::Vector3d& position,
                // std::vector<std::vector<double>>& jointPos) override;

  bool saveTwistsToFiles(const std::string& desiredTwistFilename,
                         const std::string& deltaTwistFilename,
                         const std::string& twistFilename) override;

protected:
  state_representation::CartesianState commandState_;
  state_representation::CartesianState feedbackState_;
  std::shared_ptr<controllers::IController<state_representation::CartesianState>> twistCtrl_;
  std::list<std::shared_ptr<state_representation::ParameterInterface>> parameters_;

private:
  std::vector<Eigen::VectorXd> savedDesiredTwists;
  std::vector<Eigen::VectorXd> savedDeltaTwistFromWrenchTransfroms;
  std::vector<Eigen::VectorXd> savedTwists;
};
