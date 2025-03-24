#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include <Eigen/Dense>
#include <string>
#include <vector>

double calculateEuclideanError(const std::vector<double>& desired, const std::vector<double>& actual);

bool saveToCSV(const std::vector<std::vector<double>>& data, const std::string& filename, const std::string& headers);

Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw);

std::pair<Eigen::Quaterniond, Eigen::Vector3d> applyTransformation(
    const std::pair<Eigen::Quaterniond, Eigen::Vector3d>& inputPair,
    const Eigen::Quaterniond& rotationQuat,
    const Eigen::Vector3d& translationVec);

std::pair<Eigen::Quaterniond, Eigen::Vector3d> undoTransformation(
    const std::pair<Eigen::Quaterniond, Eigen::Vector3d>& transformedPair,
    const Eigen::Quaterniond& appliedRotation,
    const Eigen::Vector3d& appliedTranslation);

double angular_distance(double angle1, double angle2);

double total_angular_distance(const std::vector<double>& v1, const std::vector<double>& v2);

std::vector<double> find_closest_angular_vector(const std::vector<std::vector<double>>& vectors,
                                                const std::vector<double>& target);

void printIKs(const std::vector<std::vector<double>>& iks);

std::pair<Eigen::Quaterniond, Eigen::Vector3d> updatePose(
    const std::pair<Eigen::Quaterniond, Eigen::Vector3d>& pairActualQuatPosOriginalSpace,
    const Eigen::VectorXd& twistDesiredEigen,
    double dt);

#endif // UTILITY_FUNCTIONS_H
