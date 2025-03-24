// clang off
#include "ITaskBase.h"
// clang on
#include <chrono>

#include "RoboticArmFactory.h"

using namespace std;
using namespace Eigen;

ITaskBase::ITaskBase(double freq, string robotName) : rosFreq_(freq), robotName_(robotName) {
  nodeTask_ = std::make_shared<rclcpp::Node>("task_node");
  // Create an unique pointer for the instance of RosInterfaceNoetic
  rosInterface_ = make_shared<RosInterfaceHumble>(robotName_);
  // polygonCoverage_ = make_unique<PolygonCoverage>(rosFreq_);
  // Create an unique pointer for the instance of RosInterfaceNoetic
  RoboticArmFactory armFactory = RoboticArmFactory();
  roboticArm_ = armFactory.createRoboticArm(robotName_);
  dataPath_ = "/home/robetarme_user/ros2_ws/data";
}
// Initialize these in the constructor or initialization method
bool ITaskBase::initializeNodeAndRate() {
  try {
    loopRate_ = std::make_unique<rclcpp::Rate>(rosFreq_); // Create the rate object once
    return true;
  } catch (const std::exception &e) {
    std::cerr << "Error initializing node or rate: " << e.what() << std::endl;
    return false;
  }
}

void ITaskBase::takeConfigTask(string taskname) {
  string alternativeYamlPath = string(WP5_TASKS_DIR) + "/config/tasks_config.yaml";
  string yamlPath = string(WP5_TASKS_DIR) + "/../../config/tasks_config.yaml";
  string nameYaml = "ros2_ws/config/tasks_config.yaml";

  // Check if the alternative YAML file exists
  ifstream originalFile(yamlPath);
  if (originalFile.good()) {
    cout << "Using general YAML file: " << nameYaml << endl;
  } else {
    yamlPath = alternativeYamlPath;
    cout << "Using local YAML file: " << yamlPath << endl;
  }

  // Load parameters from YAML file
  YAML::Node config = YAML::LoadFile(yamlPath);
  YAML::Node task = config[taskname];

  // Access parameters from the YAML file
  robotName_ = task["robot_name"].as<string>();
  limitCycleSpeed_ = task["limitCycleSpeed"].as<double>();
  convRate_ = task["convRate"].as<double>();
  linearSpeed_ = task["linearSpeed"].as<double>();
  radFlow_ = task["radFlow"].as<double>();
  velocityLimit_ = task["velocityLimit"].as<double>();
  toleranceToNextPoint_ = task["toleranceToNextPoint"].as<double>();
}

bool ITaskBase::initialize() {
  bool checkInitialization = true;
  if (roboticArm_) {
    cout << "----------------------" << roboticArm_->getName()
         << " chosen and well initializate----------------------------------" << endl;
    homeJoint_ = roboticArm_->originalHomeJoint;
    startJoint_ = homeJoint_;
  } else {
    checkInitialization = false;

    cout << "Error: roboticArm_ is null." << endl;
  }

  if (!initializeNodeAndRate()) {
    cout << "Error: Could not initialize node and rate." << endl;
    checkInitialization = false;
  }
  return checkInitialization;
}

//----------------------------------------------------
double ITaskBase::getRosFrequency_() const { return rosFreq_; }

vector<double> ITaskBase::getHomeJoint() const { return homeJoint_; }

void ITaskBase::setHomeJoint_(vector<double> desiredJoint) { homeJoint_ = desiredJoint; }

bool ITaskBase::goJointPosition(std::vector<double> desiredJoint) {
  // Retrieve the current state of the joints
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> stateJoints = rosInterface_->receiveState();
  std::vector<double> actualJoint = std::get<0>(stateJoints);
  std::vector<double> actualJointSpeed = std::get<1>(stateJoints);
  std::vector<double> jointVelocities(actualJointSpeed.size());

  bool finished = false;
  // Generate the path to the desired joint position
  std::vector<std::vector<double>> startPath = roboticArm_->getFreePath(actualJoint, desiredJoint);

  int i = 0;

  while (rclcpp::ok() && i < static_cast<int>(startPath.size()) - 1) {
    // Spin the node to process callbacks
    rosInterface_->spinNode();

    // Retrieve the current joint positions
    std::vector<double> currentJointPosition = std::get<0>(rosInterface_->receiveState());
    std::vector<double> nextJointPosition = startPath[i];

    // Calculate the required joint velocities to move from current to next joint positions
    double timeStep = 0.1; // Time step in seconds

    for (size_t j = 0; j < jointVelocities.size(); ++j) {
      jointVelocities[j] = (nextJointPosition[j] - currentJointPosition[j]) / timeStep;
    }

    // Send the calculated joint velocities to the robot
    rosInterface_->sendState(jointVelocities);

    // Check if the robot has achieved the desired configuration with some tolerance
    if (calculateEuclideanError(nextJointPosition, std::get<0>(rosInterface_->receiveState())) < 0.1) {
      i++;
    }

    // If the robot has reached the final position, stop the movement
    if (i >= static_cast<int>(startPath.size()) - 1) {
      finished = true;
      stateJoints = rosInterface_->receiveState();
      actualJointSpeed = std::get<1>(stateJoints);
      jointVelocities = std::vector<double>(actualJointSpeed.size(), 0.0);
      double norm = 0.0;
      for (double value : actualJointSpeed) {
        norm += value * value;
      }
      // Stop the robot
      while (norm > 0.001) {
        rosInterface_->spinNode();
        rosInterface_->sendState(jointVelocities);
        stateJoints = rosInterface_->receiveState();
        actualJointSpeed = std::get<1>(stateJoints);
        norm = 0.0;
        for (double value : actualJointSpeed) {
          norm += value * value;
        }
        loopRate_->sleep();
      }
      break;
    }

    // Sleep to maintain the loop rate
    loopRate_->sleep();
  }

  return finished;
}

bool ITaskBase::goHomingPosition() {
  cout << "go to home" << endl;

  bool checkPos = goJointPosition(homeJoint_);
  if (checkPos) {
    cout << "home achieved" << endl;
  }
  return checkPos;
}
bool ITaskBase::goWorkingPosition() {
  cout << "go to first position" << endl;
  bool checkPos = goJointPosition(startJoint_);
  if (checkPos) {
    cout << "first position achieved" << endl;
  }

  rosInterface_->setFirstPosReached(checkPos);
  // rosInterface_->publishFirstPoseAchieved(success);

  return checkPos;
}

bool ITaskBase::saveFile(const vector<vector<double>> &data, const string &filename, const string &header) {
  // Save the path to a CSV file
  return saveToCSV(data, filename, header);
}

Eigen::Vector3d computeSpeedLinearDS(const Eigen::Vector3d &x, const Eigen::Vector3d &attractor, double D) {
  // Define the A matrix as a diagonal matrix with -1 on the diagonal
  Eigen::Matrix3d A = -D * Eigen::Matrix3d::Identity();

  // Compute the velocity vector
  Eigen::Vector3d velocity = A * (x - attractor);

  return velocity;
}

// bool ITaskBase::goToPoint(const Vector3d &desiredPosition, const Quaterniond &desiredOrientation) {
//   bool positionReached = false;
//   rosInterface_->spinNode();

//   while (rclcpp::ok() && !positionReached) {
//     // Receive the current state of the robot joints
//     tuple<vector<double>, vector<double>, vector<double>> stateJoints = rosInterface_->receiveState();
//     vector<double> actualJoint = get<0>(stateJoints);

//     // Get the current end-effector position and orientation using forward kinematics
//     pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFKPinocchio(actualJoint);

//     Quaterniond actualOrientation = pairActualQuatPos.first;
//     Vector3d actualPosition = pairActualQuatPos.second;

//     std::vector<double> vectorQuatPos(7);
//     vectorQuatPos[0] = actualOrientation.x();
//     vectorQuatPos[1] = actualOrientation.y();
//     vectorQuatPos[2] = actualOrientation.z();
//     vectorQuatPos[3] = actualOrientation.w();
//     vectorQuatPos[4] = actualPosition.x();
//     vectorQuatPos[5] = actualPosition.y();
//     vectorQuatPos[6] = actualPosition.z();

//     pair<int, vector<double>> pairTrack = roboticArm_->getTracIK(actualJoint, vectorQuatPos);
//     // Calculate the error between pairTrack and actualJoint
//     std::vector<double> trackJoint = pairTrack.second;
//     std::vector<double> jointError(trackJoint.size());

//     for (size_t i = 0; i < trackJoint.size(); ++i) {
//       jointError[i] = trackJoint[i] - actualJoint[i];
//     }
//     // Print the error
//     std::cout << "Joint Error: ";
//     for (const auto &error : jointError) {
//       std::cout << error << " ";
//     }
//     std::cout << std::endl;

//     // Calculate the error in position and orientation
//     Vector3d positionError = (desiredPosition - actualPosition);
//     Quaterniond orientationError = desiredOrientation * actualOrientation.conjugate();
//     Vector3d speed = computeSpeedLinearDS(actualPosition, desiredPosition, 1);

//     std::pair<Quaterniond, Vector3d> pairQuatLinerSpeedIn = std::make_pair(desiredOrientation, speed);
//     VectorXd twistDesiredEigen = polygonCoverage_->getTwistFromDS(actualOrientation, pairQuatLinerSpeedIn);

//     // Use the low-level controller to compute the desired joint state
//     vector<double> desiredJoint = roboticArm_->lowLevelController(stateJoints, twistDesiredEigen);

//     // Send the desired joint state to the robot
//     rosInterface_->sendState(desiredJoint);

//     // Check if the position error is within a small tolerance
//     if (positionError.norm() < 0.1 && orientationError.vec().norm() < 0.1) {
//       positionReached = true;
//       return positionReached;
//     }
//     // Sleep to maintain the loop rate
//     rosInterface_->spinNode();
//     loopRate_->sleep();
//   }
//   return positionReached;
// }

Eigen::Vector3d ITaskBase::getDistanceBetweenFrame() {
  // set and get desired speed
  tuple<vector<double>, vector<double>, vector<double>> stateJoints;
  stateJoints = rosInterface_->receiveState();
  vector<double> actualJoint = get<0>(stateJoints);
  pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFKPinocchio(actualJoint);
  pair<Quaterniond, Vector3d> pairActualQuatPosGeo = roboticArm_->getFKGeo(actualJoint);
  Eigen::Vector3d DistanceFrame;
  DistanceFrame << pairActualQuatPos.second - pairActualQuatPosGeo.second;
  return DistanceFrame;
}

vector<vector<double>> ITaskBase::getSortedConfigsByDistanceToLastThreeJoints(
    const vector<vector<vector<double>>> &allIks, int iMin) {
  // Vector to store the configurations and their distances
  vector<pair<double, vector<double>>> distanceConfigPairs;

  // Validate inputs
  if (allIks.empty() || iMin >= allIks.size() || allIks[iMin].empty()) {
    cerr << "Error: allIks is empty or iMin is invalid." << endl;
    return {};
  }

  // Ensure there are at least 3 joints in homeJoint_
  if (homeJoint_.size() < 3) {
    cerr << "Error: homeJoint_ must have at least 3 joints." << endl;
    return {};
  }

  // Iterate over all configurations in the specified set
  for (const auto &config : allIks[iMin]) {
    // Ensure the configuration size matches homeJoint_ size
    if (config.size() != homeJoint_.size()) {
      cerr << "Error: Configuration size does not match homeJoint_ size." << endl;
      continue;
    }

    // Compute Euclidean distance using the last three joints
    double distance = 0.0;
    for (size_t k = homeJoint_.size() - 3; k < homeJoint_.size(); ++k) {
      distance += pow(config[k] - homeJoint_[k], 2);
    }
    distance = sqrt(distance);

    // Store the distance and the configuration as a pair
    distanceConfigPairs.emplace_back(distance, config);
  }

  // Sort the pairs by distance (ascending order)
  sort(distanceConfigPairs.begin(), distanceConfigPairs.end(), [](const auto &a, const auto &b) {
    return a.first < b.first; // Sort by distance
  });

  // Extract and return only the sorted configurations
  vector<vector<double>> sortedConfigs;
  for (const auto &pair : distanceConfigPairs) {
    sortedConfigs.push_back(pair.second);
  }

  return sortedConfigs;
}

vector<double> ITaskBase::getconfigClosestToHomePosition(vector<vector<vector<double>>> allIks, int iMin) {
  vector<double> vectorToHunt;

  if (!allIks.empty() && iMin < allIks.size() && !allIks[iMin].empty()) {
    double minDistance = numeric_limits<double>::max(); // Initialize to a very large value

    for (size_t j = 0; j < allIks[iMin].size(); ++j) {
      // Compute the Euclidean distance to homeJoint_
      double distance = 0.0;
      for (size_t k = 0; k < homeJoint_.size(); ++k) {
        distance += pow(allIks[iMin][j][k] - homeJoint_[k], 2);
      }
      distance = sqrt(distance);

      // Check if this configuration is closer to homeJoint_
      if (distance < minDistance) {
        minDistance = distance;
        vectorToHunt = allIks[iMin][j];
      }
    }
  } else {
    cout << "Error: ikRecorded is empty or invalid index iMin." << endl;
  }
  return vectorToHunt;
}

// vector<vector<double>> ITaskBase::getSortedConfigsByDistanceToHome(
//     const vector<vector<vector<double>>> &allIks, int iMin) {
//   // Vector to store the configurations and their distances
//   vector<pair<double, vector<double>>> distanceConfigPairs;

//   // Validate inputs
//   if (allIks.empty() || iMin >= allIks.size() || allIks[iMin].empty()) {
//     cerr << "Error: allIks is empty or iMin is invalid." << endl;
//     return {};
//   }

//   // Iterate over all configurations in the specified set
//   for (const auto &config : allIks[iMin]) {
//     // Ensure the configuration size matches homeJoint_ size
//     if (config.size() != homeJoint_.size()) {
//       cerr << "Error: Configuration size does not match homeJoint_ size." << endl;
//       continue;
//     }

//     // Compute Euclidean distance to homeJoint_
//     double distance = 0.0;
//     for (size_t k = 0; k < homeJoint_.size(); ++k) {
//       distance += pow(config[k] - homeJoint_[k], 2);
//     }
//     distance = sqrt(distance);

//     // Store the distance and the configuration as a pair
//     distanceConfigPairs.emplace_back(distance, config);
//   }

//   // Sort the pairs by distance (ascending order)
//   sort(distanceConfigPairs.begin(), distanceConfigPairs.end(), [](const auto &a, const auto &b) {
//     return a.first < b.first; // Sort by distance
//   });

//   // Extract and return only the sorted configurations
//   vector<vector<double>> sortedConfigs;
//   for (const auto &pair : distanceConfigPairs) {
//     sortedConfigs.push_back(pair.second);
//   }

//   return sortedConfigs;
// }

vector<vector<double>> ITaskBase::getSortedjoints(const vector<vector<double>> &allIks, vector<double> targetJoint) {
  // Vector to store the configurations and their distances
  vector<pair<double, vector<double>>> distanceConfigPairs;

  // Validate inputs
  if (allIks.empty() || allIks.empty()) {
    cerr << "Error: allIks is empty or iMin is invalid." << endl;
    return {};
  }

  // Iterate over all configurations in the specified set
  for (const auto &config : allIks) {
    // Ensure the configuration size matches homeJoint_ size
    if (config.size() != targetJoint.size()) {
      cerr << "Error: Configuration size does not match homeJoint_ size." << endl;
      continue;
    }

    // Compute Euclidean distance to homeJoint_
    double distance = 0.0;
    for (size_t k = 0; k < targetJoint.size(); ++k) {
      distance += pow(config[k] - targetJoint[k], 2);
    }
    distance = sqrt(distance);

    // Store the distance and the configuration as a pair
    distanceConfigPairs.emplace_back(distance, config);
  }

  // Sort the pairs by distance (ascending order)
  sort(distanceConfigPairs.begin(), distanceConfigPairs.end(), [](const auto &a, const auto &b) {
    return a.first < b.first; // Sort by distance
  });

  // Extract and return only the sorted configurations
  vector<vector<double>> sortedConfigs;
  for (const auto &pair : distanceConfigPairs) {
    sortedConfigs.push_back(pair.second);
  }

  return sortedConfigs;
}

// tuple<vector<double>, vector<double>, vector<double>> ITaskBase::getNextState(
//     tuple<vector<double>, vector<double>, vector<double>> actualState) {
//   tuple<vector<double>, vector<double>, vector<double>> nextState;
//   vector<double> actualJoint = get<0>(actualState);
//   vector<double> actualJointSpeed = get<1>(actualState);

//   // get first position cartesian
//   pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFKPinocchio(actualJoint);

//   // add offset
//   vector<double> desiredQuatPos = {pairActualQuatPos.first.x(),
//                                    pairActualQuatPos.first.y(),
//                                    pairActualQuatPos.first.z(),
//                                    pairActualQuatPos.first.w(),
//                                    pairActualQuatPos.second[0],
//                                    pairActualQuatPos.second[1],
//                                    pairActualQuatPos.second[2]};
//   vector<double> desiredQuatPosOffset = polygonCoverage_->addOffset(desiredQuatPos, -1);

//   Vector3d PosOffset;

//   PosOffset << desiredQuatPosOffset[4], desiredQuatPosOffset[5], desiredQuatPosOffset[6];
//   pairActualQuatPos.second = PosOffset;

//   polygonCoverage_->setposQuatMapping(pairActualQuatPos);

//   pair<Quaterniond, Vector3d> pairQuatLinerSpeed = polygonCoverage_->getSpeedFromMapping();

//   VectorXd twistDesiredEigen = polygonCoverage_->projectTwistToEef(
//       polygonCoverage_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed));

//   vector<double> desiredJointSpeed = roboticArm_->lowLevelController(actualState, twistDesiredEigen);

//   vector<double> nextjoint = vector<double>(actualJoint.size(), 0.0);

//   for (int j = 0; j < actualJoint.size(); j++) {
//     nextjoint[j] = actualJoint[j] + desiredJointSpeed[j] / rosFreq_;
//   }

//   // update state
//   get<0>(nextState) = nextjoint;
//   get<1>(nextState) = desiredJointSpeed;
//   get<2>(nextState) = vector<double>(actualJoint.size(), 0.0);

//   return nextState;
// }

bool ITaskBase::checkPathError(tuple<vector<double>, vector<double>, vector<double>> actualState,
                               double tolJac = 0.01,
                               double tolMaxJointSpeed = 6.28) {
  bool successFeasibility = true;
  // check if the robot is in collision
  if (roboticArm_->checkCollision(get<0>(actualState))) { //true if collision detected
    successFeasibility = false;
    // cout << "\ncheckCollision true" << endl;
  }

  MatrixXd jac = roboticArm_->getJacobian(get<0>(actualState));
  VectorXd manipulabilty = roboticArm_->computeSingularValues(jac);
  // chck if the  jacobian in square
  double jacobianDeterminant = 1;

  if (jac.rows() == jac.cols()) {
    jacobianDeterminant = jac.determinant();
  }

  // check determinant close to 0 to avoid singularity
  if (abs(jacobianDeterminant) < tolJac) {
    successFeasibility = false;
    // cout << "\nJacobian determinant close to 0" << endl;
  }

  // check if joint speed is not to high
  for (int j = 0; j < get<1>(actualState).size(); j++) {
    if (get<1>(actualState)[j] > tolMaxJointSpeed) {
      successFeasibility = false;
      // cout << "\njoint speed too high" << endl;
    }
  }
  return successFeasibility;
}

// bool ITaskBase::checkFeasibilityTracIk() {

//   RCLCPP_INFO(nodeTask_->get_logger(), "Checking feasibility Trac IK...");

//   rosInterface_->spinNode();

//   // Get the first position in Cartesian space
//   bool successFeasibility = true;
//   std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> actualState;
//   actualState = rosInterface_->receiveState();
//   vector<double> actualJoint = std::get<0>(actualState);
//   polygonCoverage_->resetMapping();

//   // Get the first position in Cartesian space
//   std::vector<double> firstWaypointOriginalSpace = polygonCoverage_->getAttractorFromMapping();
//   std::vector<double> actualPosEef = polygonCoverage_->addOffset(firstWaypointOriginalSpace, 1);

//   // Get all IK solutions
//   std::pair<int, std::vector<std::vector<double>>> ikSolution =
//       roboticArm_->getUniqueTracIKSolutions(actualJoint, actualPosEef, 20, 10e-1);

//   if (ikSolution.first == 0) {
//     successFeasibility = false;
//     RCLCPP_ERROR(nodeTask_->get_logger(), "No IK solution found for the first position");
//     startJoint_ = homeJoint_;
//     return successFeasibility;
//   }

//   std::vector<std::vector<double>> iksFirstPosition = ikSolution.second;

//   // Sort all IK solutions based on proximity to the actual joint position
//   std::vector<std::vector<double>> vectorMinIksSorted = getSortedjoints(iksFirstPosition, actualJoint);

//   int i = 0;
//   std::vector<double> targetVectorIK = vectorMinIksSorted[i];

//   // Set initial state
//   std::get<0>(actualState) = targetVectorIK;
//   std::get<1>(actualState) = std::vector<double>(targetVectorIK.size(), 0.0);
//   std::get<2>(actualState) = std::vector<double>(targetVectorIK.size(), 0.0);
//   auto t1 = std::chrono::high_resolution_clock::now();
//   while (rclcpp::ok() && successFeasibility && !polygonCoverage_->isFinished()) {
//     // Spin the node to process callbacks
//     // rosInterface_->spinNode();

//     // Get the next step state
//     actualState = getNextState(actualState);

//     // Check for path errors (e.g., collisions, singularities, or high joint speeds)
//     successFeasibility = checkPathError(actualState);

//     if (!successFeasibility && i < static_cast<int>(vectorMinIksSorted.size()) - 1) {
//       RCLCPP_WARN(nodeTask_->get_logger(), "Feasibility test failed: Iteration %d", i);
//       successFeasibility = true;
//       i++;
//       polygonCoverage_->resetMapping();

//       // Switch to the next IK solution
//       targetVectorIK = vectorMinIksSorted[i];
//       std::get<0>(actualState) = targetVectorIK;
//       std::get<1>(actualState) = std::vector<double>(targetVectorIK.size(), 0.0);
//       std::get<2>(actualState) = std::vector<double>(targetVectorIK.size(), 0.0);
//     }
//   }

//   if (successFeasibility) {
//     startJoint_ = targetVectorIK;
//     RCLCPP_INFO(nodeTask_->get_logger(), "Feasibility test succeeded. Start joint set to first position.");
//   } else {
//     startJoint_ = homeJoint_;
//     RCLCPP_ERROR(nodeTask_->get_logger(), "Feasibility test failed. Start joint set to home position.");
//   }

//   polygonCoverage_->resetMapping();

//   return successFeasibility;
// }

// bool ITaskBase::checkFeasibilityTracIk() {

//   RCLCPP_INFO(nodeTask_->get_logger(), "Checking feasibility Trac IK...");

//   rosInterface_->spinNode();

//   // Get the first position in Cartesian space
//   bool successFeasibility = true;
//   std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> actualState;
//   actualState = rosInterface_->receiveState();
//   vector<double> actualJoint = std::get<0>(actualState);
//   polygonCoverage_->resetMapping();

//   // Get the first position in Cartesian space
//   std::vector<double> firstWaypointOriginalSpace = polygonCoverage_->getAttractorFromMapping();
//   std::vector<double> actualPosEef = polygonCoverage_->addOffset(firstWaypointOriginalSpace, 1);

//   // Get all IK solutions
//   std::pair<int, std::vector<std::vector<double>>> ikSolution =
//       roboticArm_->getUniqueTracIKSolutions(actualJoint, actualPosEef, 20, 10e-1);

//   if (ikSolution.first == 0) {
//     successFeasibility = false;
//     RCLCPP_ERROR(nodeTask_->get_logger(), "No IK solution found for the first position");
//     startJoint_ = homeJoint_;
//     return successFeasibility;
//   }

//   std::vector<std::vector<double>> iksFirstPosition = ikSolution.second;

//   // Sort all IK solutions based on proximity to the actual joint position
//   std::vector<std::vector<double>> vectorMinIksSorted = getSortedjoints(iksFirstPosition, actualJoint);

//   // compute entire cartesian path
//   Vector3d initialpos(firstWaypointOriginalSpace[4], firstWaypointOriginalSpace[5], firstWaypointOriginalSpace[6]);
//   Quaterniond initialquat(firstWaypointOriginalSpace[3],
//                           firstWaypointOriginalSpace[0],
//                           firstWaypointOriginalSpace[1],
//                           firstWaypointOriginalSpace[2]);
//   pair<Quaterniond, Vector3d> pairActualQuatPos = make_pair(initialquat, initialpos);
//   pair<Quaterniond, Vector3d> pairActualQuatPosEef = polygonCoverage_->addOffset(pairActualQuatPos, -1);

//   // vector<pair<Quaterniond, Vector3d>> CartesianPathTarget;
//   vector<pair<Quaterniond, Vector3d>> CartesianPathEef;
//   // CartesianPathTarget.push_back(pairActualQuatPos);
//   CartesianPathEef.push_back(pairActualQuatPosEef);

//   while (rclcpp::ok() && !polygonCoverage_->isFinished()) {

//     polygonCoverage_->setposQuatMapping(pairActualQuatPos);

//     pair<Quaterniond, Vector3d> pairQuatLinerSpeed = polygonCoverage_->getSpeedFromMapping();

//     Vector3d nextPos = pairActualQuatPos.second + pairQuatLinerSpeed.second * 10 / rosFreq_;
//     pairActualQuatPos = make_pair(pairQuatLinerSpeed.first, nextPos);
//     // CartesianPathTarget.push_back(pairActualQuatPos);

//     pair<Quaterniond, Vector3d> pairActualQuatPosEef = polygonCoverage_->addOffset(pairActualQuatPos, -1);
//     CartesianPathEef.push_back(pairActualQuatPosEef);
//   }
//   polygonCoverage_->resetMapping();

//   int i = 0;
//   int mod = 0;
//   std::vector<double> targetVectorIK = vectorMinIksSorted[i];
//   actualJoint = vectorMinIksSorted[i];
//   while (rclcpp::ok() && successFeasibility && mod < CartesianPathEef.size()) {

//     // Check for path errors (e.g., collisions, singularities, or high joint speeds)
//     successFeasibility = checkPathError(actualState);
//     // Get the next step state
//     vector<double> QuatPosCartesianPathTargeteef = {CartesianPathEef[mod].first.x(),
//                                                     CartesianPathEef[mod].first.y(),
//                                                     CartesianPathEef[mod].first.z(),
//                                                     CartesianPathEef[mod].first.w(),
//                                                     CartesianPathEef[mod].second[0],
//                                                     CartesianPathEef[mod].second[1],
//                                                     CartesianPathEef[mod].second[2]};

//     pair<int, vector<double>> result = roboticArm_->getTracIK(actualJoint, QuatPosCartesianPathTargeteef);
//     actualJoint = result.second;
//     mod += 10;

//     if (!successFeasibility && i < static_cast<int>(vectorMinIksSorted.size()) - 1) {
//       RCLCPP_WARN(nodeTask_->get_logger(), "Feasibility test failed: Iteration %d", i);
//       successFeasibility = true;
//       i++;
//       // Switch to the next IK solution
//       actualJoint = vectorMinIksSorted[i];
//       targetVectorIK = vectorMinIksSorted[i];
//     }
//   }

//   if (successFeasibility) {
//     startJoint_ = targetVectorIK;
//     RCLCPP_INFO(nodeTask_->get_logger(), "Feasibility test succeeded. Start joint set to first position.");
//   } else {
//     startJoint_ = homeJoint_;
//     RCLCPP_ERROR(nodeTask_->get_logger(), "Feasibility test failed. Start joint set to home position.");
//   }

//   return successFeasibility;
// }
bool ITaskBase::moveBackEEF(double distance) {
  // RCLCPP_INFO(nodeTask_->get_logger(), "Starting to move back the EEF by a distance of %f", distance);

  int i = 0;
  bool check = true;

  rclcpp::Rate loopRate(rosFreq_); // Get the loop frequency dynamically
  int totalSteps = rosFreq_;       // Assuming you want the motion to be distributed over 'rosFreq_' steps

  while (rclcpp::ok() && i < totalSteps) {
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> stateJoints;
    Eigen::VectorXd twistDesiredEigen = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd deltaTwist = Eigen::VectorXd::Zero(6);

    // Calculate the current movement distance using linear interpolation
    // Moving forward (0 to distance) then backward (distance to 0)
    double progress = (double) i / (double) totalSteps;
    double currentDistance = 0.0;

    if (progress <= 0.5) {
      // Move from 0 to distance
      currentDistance = progress * 2 * distance; // Linear interpolation: 0 -> distance
    } else {
      // Move back from distance to 0
      currentDistance = distance - ((progress - 0.5) * 2 * distance); // Linear interpolation: distance -> 0
    }

    // Set the desired movement along the Z-axis
    deltaTwist(2) = -1 * currentDistance;

    // Get the current state of the robot joints
    stateJoints = rosInterface_->receiveState();

    // Calculate the desired joint speeds
    std::vector<double> desiredJointSpeed =
        roboticArm_->lowLevelControllerSF(stateJoints, twistDesiredEigen, deltaTwist);

    // Send the desired joint speeds to the robot
    rosInterface_->sendState(desiredJointSpeed);

    i++;

    // Spin the Node to process callbacks
    rosInterface_->spinNode();

    // Sleep to maintain the loop rate
    loopRate.sleep();
  }
  return check;
}

// bool ITaskBase::checkFeasibility() {
//   if (roboticArm_->checkIkGeoUp())
//     return checkFeasibilityGeo();
//   else
//     return checkFeasibilityTracIk();
// }
// bool ITaskBase::checkFeasibilityGeo() {

//   RCLCPP_INFO(nodeTask_->get_logger(), "Checking feasibility Ik GEO...");

//   bool successFeasibility = true;
//   int iter = 0;
//   int iMin = 0;
//   int lenIK = 16;
//   std::vector<std::vector<double>> iks;
//   std::vector<std::vector<std::vector<double>>> ikRecorded;
//   std::pair<Eigen::Quaterniond, Eigen::Vector3d> pairActualQuatPosOriginalSpace;
//   std::pair<Eigen::Quaterniond, Eigen::Vector3d> pairActualQuatPosEef;

//   Eigen::Quaterniond rotationQuat = rpyToQuaternion(0.0, 0.0, 0.0);
//   Eigen::Vector3d translationVec = getDistanceBetweenFrame();

//   polygonCoverage_->resetMapping();

//   // Get the first waypoint in Cartesian space
//   std::vector<double> firstWaypointOriginalSpace = polygonCoverage_->getAttractorFromMapping();
//   std::vector<double> actualPosEef = polygonCoverage_->addOffset(firstWaypointOriginalSpace, 1);

//   Eigen::Quaterniond actualQuatEef(actualPosEef[3], actualPosEef[0], actualPosEef[1], actualPosEef[2]);
//   Eigen::Vector3d posOffsetEef(actualPosEef[4], actualPosEef[5], actualPosEef[6]);
//   pairActualQuatPosEef = {actualQuatEef, posOffsetEef};

//   auto transformActualQuatEef = undoTransformation(pairActualQuatPosEef, rotationQuat, translationVec);
//   bool checkIk = roboticArm_->getIKGeo(transformActualQuatEef.first, transformActualQuatEef.second, iks);

//   if (!checkIk) {
//     RCLCPP_ERROR(nodeTask_->get_logger(), "No IK solutions found.");
//     startJoint_ = homeJoint_;
//     return false;
//   }

//   ikRecorded.push_back(iks);
//   if (iks.size() < lenIK) {
//     lenIK = iks.size();
//     iMin = iter;
//   }
//   iter++;

//   while (rclcpp::ok() && !polygonCoverage_->isFinished()) {
//     rosInterface_->spinNode(); // Process callbacks

//     pairActualQuatPosOriginalSpace = polygonCoverage_->addOffset(pairActualQuatPosEef, -1);
//     polygonCoverage_->setposQuatMapping(pairActualQuatPosOriginalSpace);

//     auto pairQuatLinerSpeed = polygonCoverage_->getSpeedFromMapping();
//     Eigen::VectorXd twistDesiredEigen = polygonCoverage_->projectTwistToEef(
//         polygonCoverage_->getTwistFromDS(pairActualQuatPosOriginalSpace.first, pairQuatLinerSpeed));

//     pairActualQuatPosEef = updatePose(pairActualQuatPosEef, twistDesiredEigen, 1 / getRosFrequency_());
//     transformActualQuatEef = undoTransformation(pairActualQuatPosEef, rotationQuat, translationVec);

//     checkIk = roboticArm_->getIKGeo(transformActualQuatEef.first, transformActualQuatEef.second, iks);
//     if (!checkIk) {
//       RCLCPP_ERROR(nodeTask_->get_logger(), "No IK solutions found during iteration.");
//       startJoint_ = homeJoint_;
//       return false;
//     }
//     ikRecorded.push_back(iks);
//     if (iks.size() < lenIK) {
//       lenIK = iks.size();
//       iMin = iter;
//     }
//   }

//   // Process the IK solutions recorded
//   std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> actualState;
//   std::vector<std::vector<double>> firstPositionIkFeasible;
//   std::vector<std::vector<double>> minPositionIkFeasible = ikRecorded[iMin];

//   for (int i = 0; i < static_cast<int>(ikRecorded[iMin].size()); i++) {
//     int j = iMin;
//     std::vector<double> vectorToHunt = ikRecorded[j][i];
//     while (rclcpp::ok() && j != 0) {
//       j--;
//       vectorToHunt = find_closest_angular_vector(ikRecorded[j], vectorToHunt);
//     }
//     firstPositionIkFeasible.push_back(vectorToHunt);
//   }

//   std::vector<std::vector<double>> vectorFirstIksSorted = getSortedjoints(firstPositionIkFeasible, homeJoint_);
//   int i = 0;
//   std::vector<double> targetVectorIK = vectorFirstIksSorted[i];

//   // Set the initial state
//   std::get<0>(actualState) = targetVectorIK;
//   std::get<1>(actualState) = std::vector<double>(targetVectorIK.size(), 0.0);
//   std::get<2>(actualState) = std::vector<double>(targetVectorIK.size(), 0.0);

//   while (rclcpp::ok() && successFeasibility && !polygonCoverage_->isFinished()) {
//     rosInterface_->spinNode(); // Process callbacks

//     // Get the next step state
//     actualState = getNextState(actualState);

//     // Check for path errors (e.g., collisions, singularities, or high joint speeds)
//     successFeasibility = checkPathError(actualState);

//     if (!successFeasibility && i < static_cast<int>(vectorFirstIksSorted.size()) - 1) {
//       RCLCPP_WARN(nodeTask_->get_logger(), "Feasibility test failed: Iteration %d", i);
//       successFeasibility = true;
//       i++;
//       polygonCoverage_->resetMapping();
//       targetVectorIK = vectorFirstIksSorted[i];
//       std::get<0>(actualState) = targetVectorIK;
//       std::get<1>(actualState) = std::vector<double>(targetVectorIK.size(), 0.0);
//       std::get<2>(actualState) = std::vector<double>(targetVectorIK.size(), 0.0);
//     }
//   }

//   if (successFeasibility) {
//     startJoint_ = targetVectorIK;
//     RCLCPP_INFO(nodeTask_->get_logger(), "Feasibility test succeeded. Start joint set to target.");
//   } else {
//     startJoint_ = homeJoint_;
//     RCLCPP_ERROR(nodeTask_->get_logger(), "Feasibility test failed. Start joint set to home position.");
//   }

//   polygonCoverage_->resetMapping();

//   return successFeasibility;
// }
