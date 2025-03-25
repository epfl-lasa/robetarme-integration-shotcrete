#pragma once

/**
 * @file
 * @brief This file contains the declaration of the ITaskBase class and its
 * associated enums and dependencies.
 */

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <yaml-cpp/yaml.h>

#include <map>
#include <memory>

#include "IRoboticArmBase.h"
#include "RosInterfaceHumble.hpp"
// #include "TaskShotcreteKUL.h"
// #include "ToolsShotcrete.h"
// #include "ToolsSurfaceFinishing.h"
#include "utilityFunctions.h"

// #include "visualization_msgs/Marker.h"

/**
 * @brief Enum representing different types of tasks.
 */
enum TaskType : int8_t {
  TASK_UNDEFINED = -1, /**< Undefined task type. */
  SHOTCRETE,           /**< Shotcrete task type. */
  SHOTCRETEKUL,        /**< Shotcrete task type. */
  SURFACE_FINISHING,   /**< Surface finishing task type. */
  SAND_BLASTING,       /**< Sand blasting task type. */
  MAM_REBARS,          /**< MAM rebar task type. */
  NB_TASKS             /**< Number of task types. Keep at the end of enum. */
};

/**
 * @brief Base class for tasks.
 */
class ITaskBase {

public:
  /**
   * @brief Map of task names to TaskType enums.
   */
  inline static const std::map<std::string, TaskType> taskTypes{
      {"shotcrete", SHOTCRETE}, {"shotcreteKUL", SHOTCRETEKUL}, {"surface_finishing", SURFACE_FINISHING}};

  /**
   * @brief Flag indicating if initialization has been checked.
   */
  bool checkInitialization = false;

  /**
   * @brief Flag indicating if task completion has been checked.
   */
  bool checkFinish = false;

  /**
   * @brief Flag indicating if path computation has been checked.
   */
  bool checkPath = false;

  /**
   * @brief Flag indicating if homing position has been checked.
   */
  bool checkHomingPosition = false;

  /**
   * @brief Flag indicating if working position has been checked.
   */
  bool checkWorkingPosition = false;

  /**
   * @brief Constructor.
   * @param freq Frequency for ROS loop.
   * @param robotName Name of the robot.
   */
  ITaskBase(double freq, std::string robotName);

  /**
   * @brief Initializes the task.
   * @return True if initialization is successful, false otherwise.
   */
  bool initialize();

  /**
   * @brief Computes the path for the task.
   * @return True if path computation is successful, false otherwise.
   */
  virtual bool computePath() = 0;

  /**
   * @brief go to human sharing.
   * @return True if human-shared, false otherwise.
   */
  virtual bool humanSharing() = 0;

  /**
   * @brief Executes the task.
   * @return True if execution is successful, false otherwise.
   */
  virtual bool execute() = 0;

  /**
   * @brief Moves to the homing position.
   * @return True if successful, false otherwise.
   */
  virtual bool goHomingPosition();

  /**
   * @brief Moves to the working position.
   * @return True if successful, false otherwise.
   */
  virtual bool goWorkingPosition();

  // /**
  //  * @brief Gets the ROS loop rate by reference.
  //  * @return ROS loop rate.
  //  */
  // ros::Rate *getRosLoopRate_();

  bool moveBackEEF(double distance);

  /**
   * @brief Gets the home joint configuration.
   * @return Home joint configuration.
   */
  std::vector<double> getHomeJoint() const;

protected:
  /**
   * @brief Gets the ROS frequency.
   * @return ROS frequency.
   */
  double getRosFrequency_() const;

  // /**
  //  * @brief Gets the ROS node handle.
  //  * @return ROS node handle.
  //  */
  // ros::NodeHandle getRosNodehandle_() const;

  /**
   * @brief Sets the home joint configuration.
   * @param desiredJoint Desired joint configuration.
   */
  void setHomeJoint_(std::vector<double> desiredJoint);

  /**
   * @brief Pointer to IRoboticArmBase instance.
   */
  std::unique_ptr<IRoboticArmBase> roboticArm_ = nullptr;

  // /**
  //  * @brief Pointer to IToolsBase instance.
  //  */
  // std::unique_ptr<IToolsBase> tools_ = nullptr;

  /**
   * @brief Pointer to RosInterfaceNoetic instance.
   */
  std::shared_ptr<RosInterfaceHumble> rosInterface_ = nullptr;
  /**
  //  * @brief Pointer to polygonCoverage instance.
  //  */
  // std::unique_ptr<PolygonCoverage> polygonCoverage_ = nullptr;

  /**
   * @brief Moves to a specific point with a linear DS.
   * @param firstQuatPosOffset Vector representing the position offset.
   * @return True if successful, false otherwise.
   */
  // bool goToPoint(const Eigen::Vector3d &desiredPosition, const Eigen::Quaterniond &desiredOrientation);
  bool goJointPosition(std::vector<double> desiredJoint);
  bool checkSharedControl_;

  /**
   * @brief Take the configuration of the task.
   * @param taskname Name of the task.
   * @return  void
   */
  void takeConfigTask(std::string taskname);

  bool saveFile(const std::vector<std::vector<double>> &data, const std::string &filename, const std::string &header);

  std::string robotName_;
  double limitCycleSpeed_;
  double convRate_;
  double linearSpeed_;
  double radFlow_;
  double offsetTool_;
  double offsetTarget_;
  double velocityLimit_;
  double toleranceToNextPoint_;

  /**
   * @brief Home joint configuration.
   */
  std::vector<double> homeJoint_;

  std::vector<double> startJoint_;

  /**
   * @brief ROS frequency.
   */
  double rosFreq_;
  // bool checkFeasibility();
  // bool checkFeasibilityGeo();
  // bool checkFeasibilityTracIk();

  std::vector<double> getconfigClosestToHomePosition(std::vector<std::vector<std::vector<double>>> allIks, int iMin);
  std::vector<std::vector<double>> getSortedConfigsByDistanceToLastThreeJoints(
      const std::vector<std::vector<std::vector<double>>> &allIks, int iMin);
  std::vector<std::vector<double>> getSortedjoints(const std::vector<std::vector<double>> &allIks,
                                                   std::vector<double> targetJoint);
  // std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getNextState(
  //     std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> actualState);
  bool checkPathError(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> actualState,
                      double tolJac,
                      double tolMaxJointSpeed);

  std::unique_ptr<rclcpp::Rate> loopRate_;
  std::shared_ptr<rclcpp::Node> nodeTask_; // Shared Node instance
  std::string dataPath_ = "";

private:
  // /**
  //  * @brief ROS node handle.
  //  */
  // ros::NodeHandle nh_;

  // /**
  //  * @brief ROS loop rate.
  //  */
  // ros::Rate loopRate_;

  // Eigen::Vector3d getDistanceBetweenFrame();
  bool initializeNodeAndRate();
};
