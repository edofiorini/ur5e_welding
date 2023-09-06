#pragma once
#include <Eigen/Dense>

namespace ur_ros
{
    const int dof = 6;
    const std::array<double, dof> dh_d = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
    const std::array<double, dof> dh_m = {3.761, 8.058, 2.846, 1.37, 1.3, 0.365};
    const std::array<double, dof> dh_alpha = {M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0};
    const std::array<double, dof> dh_a = {0, -0.42500, -0.39225, 0, 0, 0};

    Eigen::Matrix<double, 3, 1> rotMToeulerZYZ(Eigen::Matrix<double, 3, 3> R);
    Eigen::Matrix<double, 3, 1> quatToeulerZYZ(Eigen::Matrix<double, 4, 1> q);
    Eigen::Matrix<double, 3, 3> eulerZYZToRotM(Eigen::Matrix<double, 3, 1> euler);

    Eigen::Matrix<double, 4, 4> getTransformation(int from, int to, Eigen::Matrix<double, dof, 1> q);
    Eigen::Matrix<double, 6, 1> forwardKinematics(Eigen::Matrix<double, dof, 1> q);
    Eigen::Matrix<double, dof, 1> inverseKinematics(const Eigen::Matrix<double, 6, 1> &x, int index);
    Eigen::Matrix<double, dof, dof> getJacobian(Eigen::Matrix<double, dof, 1> q);
    Eigen::Matrix<double, dof, dof> getAnalyticalJacobian(Eigen::Matrix<double, dof, 1> q);
}