/**
 * @file IRoboticArmBase.h
 * @brief Declaration of the IRoboticArmBase class
 * @version 0.1
 * @date 2024-03-07
 * @copyright Copyright (c) 2024 - EPFL
 */

#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <robot_model/Model.hpp>
#include <sstream>
#include <string>
#include <trac_ik/trac_ik.hpp> // Include the TRAC_IK header
#include <vector>

#include "controllers/ControllerFactory.hpp"
#include "state_representation/parameters/ParameterInterface.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/joint/JointState.hpp"

/**
 * @brief Mother class to create all the prototype functions needed in different
 * robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary
 * functions to control it.
 */
class IRoboticArmBase {
public:
  /**
   * @brief Default constructor for IRoboticArmBase.
   */
  IRoboticArmBase() = default;

  /**
   * @brief Destructor for IRoboticArmBase.
   */
  virtual ~IRoboticArmBase() = default;

  /**
   * @brief Get the name of the robotic arm.
   * @return Name of the robotic arm.
   */
  std::string getName() const { return robotName_; }

  /**
   * @brief Get the number of joints of the robotic arm.
   * @return Number of joints of the robotic arm.
   */
  size_t getNbJoints() const { return jointNames_.size(); }

  /**
   * @brief Get forward kinematics of the robotic arm.
   *
   * @param jointPos Vector of joint positions.
   * @return Pair representing the Cartesian pose (quaternion and position) of
   * the end effector.
   */
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFKPinocchio(const std::vector<double> &jointPos) const;
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFKPinocchio(const std::vector<double> &jointPos,
                                                                const std::string &tipLink) const;
  bool setTransformEigen(const Eigen::Affine3d &transform);

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   *
   * @param quaternion Quaternion of the end effector.
   * @param position Position of the end effector.
   * @param jointPos Vector of joint positions.
   * @return Vector representing the desired joint positions.
   */
  std::vector<double> getIKPinocchio(const Eigen::Quaterniond &quaternion,
                                     const Eigen::Vector3d &position,
                                     const std::vector<double> &jointPos) const;

  /**
   * @brief Get the twist (linear and angular velocities) of the robotic arm.
   *
   * @param posJoint Vector of joint positions.
   * @param speedJoint Vector of joint velocities.
   * @return Vector representing the twist (linear and angular velocities) of
   * the end effector.
   */
  Eigen::VectorXd getTwistFromJointState(const std::vector<double> &posJoint,
                                         const std::vector<double> &speedJoint) const;

  /**
   * @brief Get the Jacobian matrix of the robotic arm.
   *
   * @param jointPos Vector of joint positions.
   * @return Matrix representing the Jacobian matrix of the end effector.
   */
  Eigen::MatrixXd getJacobian(const std::vector<double> &jointPos) const;

  /**
   * @brief Get the inverse dynamics of the robotic arm.
   *
   * @param jointPos Vector of joint positions.
   * @param speedEigen Vector of twist desired.
   * @return Vector representing the joint torques required for the given joint
   * positions and velocities.
   */
  std::vector<double> getInvertVelocities(const std::vector<double> &jointPos, const Eigen::VectorXd &speedEigen) const;

  /**
   * @brief Low-level controller function for the robotic arm.
   *
   * This method implements the low-level controller functionality for the
   * robotic arm for shotcrete.
   * @param data Tuple containing vectors of joint positions, joint velocities,
   * and joint efforts.
   * @param twist Vector representing the twist for the end effector.
   * @return Vector containing the control commands for the robotic arm.
   */
  virtual std::vector<double> lowLevelController(
      const std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> &data,
      const Eigen::VectorXd &twist) = 0;

  /**
   * @brief Low-level controller function for the robotic arm with a safety
   * factor.
   *
   * This method implements the low-level controller functionality for the
   * robotic arm for surface finishing.
   * @param data Tuple containing vectors of joint positions, joint velocities,
   * and joint efforts.
   * @param twist Vector representing the twist for the end effector.
   * @param deltaTwist Vector representing the delta twist for the end effector.
   * @return Vector containing the control commands for the robotic arm.
   */
  virtual std::vector<double> lowLevelControllerSF(
      const std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> &data,
      const Eigen::VectorXd &twist,
      const Eigen::VectorXd &deltaTwist) = 0;

  /**
   * @brief Original home joint positions of the robotic arm.
   */
  std::vector<double> originalHomeJoint = {};

  Eigen::VectorXd computeSingularValues(const Eigen::MatrixXd &J) const;

  // You can keep this function if it's a part of the common interface
  // virtual bool getIKGeo(const Eigen::Quaterniond &quaternion,
  //                       const Eigen::Vector3d &position,
  //                       std::vector<std::vector<double>> &jointPos) {
  //   // Default implementation (if necessary)
  //   return false; // Return false for non-UR5 arms
  // }
  // virtual std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFKGeo(const std::vector<double> &jointPos) = 0;

  virtual bool saveTwistsToFiles(const std::string &desiredTwistFilename,
                                 const std::string &deltaTwistFilename,
                                 const std::string &twistFilename) {
    // Default implementation (if necessary)
    return false; // Return false for non-UR5 arms
  }

  //track-ik
  void initTracIK();
  std::pair<int, std::vector<double>> getTracIK(const std::vector<double> &actualJoint,
                                                const std::vector<double> &vectorQuatPos) const;
  std::pair<bool, std::vector<std::vector<double>>> getUniqueTracIKSolutions(const std::vector<double> &actualJoint,
                                                                             const std::vector<double> &vectorQuatPos,
                                                                             int n,
                                                                             double tolerance) const;
  //pybullet
  std::vector<std::vector<double>> getFreePath(const std::vector<double> &start_configuration,
                                               const std::vector<double> &target_configuration) const;
  bool checkCollision(const std::vector<double> &jointConfig) const;
  std::vector<std::pair<double, double>> getJointLimits() const;
  // bool checkIkGeoUp() const;

protected:
  /**
   * @brief Initialization function for inverse kinematics.
   */
  // Protected members
  std::string robotName_ = "";
  std::string nameSpace_ = "";
  std::vector<std::string> jointNames_;
  std::string baseLink_ = "";
  std::string tipLink_ = "";
  std::string tipJoint_ = "";
  std::string referenceFrame_ = "";
  std::string pathUrdf_ = "";
  std::unique_ptr<robot_model::Model> model_;
  Eigen::VectorXd twist_;
  // bool ikGeoUp_ = false;
  TRAC_IK::TRAC_IK *ikSolver; // Pointer to the IK solver
  Eigen::Affine3d transformEigen_;

  int nJoint_ = 0;
  struct robot_model::InverseKinematicsParameters paramsIK_ = {};
  Eigen::VectorXd transformWrenchToBase_(const Eigen::VectorXd &wrench_end_effector,
                                         const Eigen::Vector3d &position_end_effector,
                                         const Eigen::Quaterniond &orientation_end_effector_to_base) const;
  std::unique_ptr<pybind11::object> self_avoidance_instance_;
  void initializeSelfAvoidance(const std::string &urdf_path, const std::vector<std::string> &obstacle_stl_path);
  KDL::Chain chain_{};

private:
};
