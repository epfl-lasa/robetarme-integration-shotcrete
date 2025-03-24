#include "utilityFunctions.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;

// Function definitions
vector<vector<double>> interpolateJointConfigurations(const vector<double>& q_start,
                                                      const vector<double>& q_end,
                                                      int n_steps) {
  vector<vector<double>> interpolated_configs(n_steps, vector<double>(q_start.size()));

  for (int j = 0; j < n_steps; ++j) {
    double ratio = static_cast<double>(j) / (n_steps - 1);
    for (size_t i = 0; i < q_start.size(); ++i) {
      interpolated_configs[j][i] = q_start[i] + ratio * (q_end[i] - q_start[i]);
    }
  }

  return interpolated_configs;
}

double calculateEuclideanError(const vector<double>& desired, const vector<double>& actual) {
  if (desired.size() != actual.size()) {
    cerr << "Error: Vectors must be of the same length." << endl;
    return -1;
  }

  double sumSquaredDifferences = 0.0;
  for (size_t i = 0; i < desired.size(); ++i) {
    double difference = desired[i] - actual[i];
    sumSquaredDifferences += difference * difference;
  }

  return sqrt(sumSquaredDifferences);
}

bool saveToCSV(const vector<vector<double>>& data, const string& filename, const string& header) {
  ofstream file(filename);

  if (!file.is_open()) {
    cerr << "Failed to open file: " << filename << endl;
    return false;
  }

  // Write the header
  if (!header.empty()) {
    file << header << "\n"; // Directly write the string as a single line
  }

  // Write the data
  for (const auto& row : data) {
    for (size_t i = 0; i < row.size(); ++i) {
      file << row[i];
      if (i < row.size() - 1) {
        file << ",";
      }
    }
    file << "\n";
  }

  file.close();
  return true;
}

Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q;
}

pair<Eigen::Quaterniond, Eigen::Vector3d> applyTransformation(
    const pair<Eigen::Quaterniond, Eigen::Vector3d>& inputPair,
    const Eigen::Quaterniond& rotationQuat,
    const Eigen::Vector3d& translationVec) {
  Eigen::Quaterniond newQuat = rotationQuat * inputPair.first;
  Eigen::Vector3d newVec = rotationQuat * inputPair.second + translationVec;

  return make_pair(newQuat, newVec);
}

pair<Eigen::Quaterniond, Eigen::Vector3d> undoTransformation(
    const pair<Eigen::Quaterniond, Eigen::Vector3d>& transformedPair,
    const Eigen::Quaterniond& appliedRotation,
    const Eigen::Vector3d& appliedTranslation) {
  Eigen::Quaterniond inverseQuat = appliedRotation.conjugate();
  Eigen::Vector3d originalVec = inverseQuat * (transformedPair.second - appliedTranslation);
  Eigen::Quaterniond originalQuat = inverseQuat * transformedPair.first;

  return make_pair(originalQuat, originalVec);
}

double angular_distance(double angle1, double angle2) {
  double diff = fmod(angle1 - angle2 + 360.0, 360.0);
  return min(diff, 360.0 - diff);
}

double total_angular_distance(const vector<double>& v1, const vector<double>& v2) {
  double total_distance = 0.0;
  for (size_t i = 0; i < v1.size(); ++i) {
    total_distance += angular_distance(v1[i], v2[i]);
  }
  return total_distance;
}

vector<double> find_closest_angular_vector(const vector<vector<double>>& vectors, const vector<double>& target) {
  double min_distance = numeric_limits<double>::max();
  vector<double> closest_vector;

  for (const auto& vec : vectors) {
    double dist = total_angular_distance(vec, target);
    if (dist < min_distance) {
      min_distance = dist;
      closest_vector = vec;
    }
  }

  return closest_vector;
}

void printIKs(const vector<vector<double>>& iks) {
  cout << "Printing IK solutions:" << endl;
  for (size_t i = 0; i < iks.size(); ++i) {
    cout << "IK Solution " << i << ": ";
    for (size_t j = 0; j < iks[i].size(); ++j) {
      cout << iks[i][j] << " ";
    }
    cout << endl;
  }
}

pair<Eigen::Quaterniond, Eigen::Vector3d> updatePose(
    const pair<Eigen::Quaterniond, Eigen::Vector3d>& pairActualQuatPosOriginalSpace,
    const Eigen::VectorXd& twistDesiredEigen,
    double dt) {
  // Extract linear and angular velocities from the twist
  Eigen::Vector3d linear_velocity = twistDesiredEigen.head<3>();
  Eigen::Vector3d angular_velocity = twistDesiredEigen.tail<3>();

  // Update position using linear velocity
  Eigen::Vector3d new_position = pairActualQuatPosOriginalSpace.second + linear_velocity * dt;

  // Update orientation using angular velocity
  Eigen::AngleAxisd delta_rotation(angular_velocity.norm() * dt, angular_velocity.normalized());
  Eigen::Quaterniond delta_quaternion(delta_rotation);
  Eigen::Quaterniond new_orientation = pairActualQuatPosOriginalSpace.first * delta_quaternion;
  new_orientation.normalize(); // Normalize the quaternion

  // Return the updated quaternion and position as a new pair
  return make_pair(new_orientation, new_position);
}
