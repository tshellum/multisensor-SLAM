#pragma once

/*** Eigen packages ***/
#include <Eigen/Geometry> 
#include <Eigen/Dense>

/*** GTSAM packages ***/
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 


class MotionModel
{
private:
  const int dim_;
  Eigen::MatrixXd base_;

public:
  MotionModel() 
  : dim_(6),
    base_(Eigen::MatrixXd::Identity(dim_, dim_))
  {
    base_.topRightCorner(dim_/2, dim_/2) = Eigen::MatrixXd::Identity(dim_/2, dim_/2);
  };

  ~MotionModel() {};

  Eigen::Affine3d calculateRelativePose(Eigen::Affine3d T_prev, Eigen::Affine3d T_cur);
  Eigen::Vector3d approximateVelocity(Eigen::Affine3d T_prev, Eigen::Affine3d T_cur, double dt);
  Eigen::Vector3d predictTranslation(double dt, Eigen::Vector3d angle_k, Eigen::Vector3d vel_k);
  Eigen::Affine3d predictPose(Eigen::Affine3d T_prev, Eigen::Affine3d T_cur, double dt);
};


Eigen::Affine3d MotionModel::calculateRelativePose(Eigen::Affine3d T_prev, Eigen::Affine3d T_cur)
{
  return T_prev.inverse() * T_cur;
}


Eigen::Vector3d MotionModel::approximateVelocity(Eigen::Affine3d T_prev, Eigen::Affine3d T_cur, double dt)
{
  Eigen::Affine3d pose_relative = calculateRelativePose(T_prev, T_cur);
  return Eigen::Vector3d(pose_relative.translation() / dt);
}


Eigen::Vector3d MotionModel::predictTranslation(double dt, Eigen::Vector3d ang_k, Eigen::Vector3d vel_k)
{
  double yaw = ang_k.coeff(2);
  Eigen::MatrixXd update_step = base_;
  update_step.block<2,2>(0,3) = Eigen::Scaling(dt) * Eigen::Rotation2Dd(yaw);

  std::cout << "base_ matrix m:\n" << update_step << std::endl;

  Eigen::VectorXd states = Eigen::VectorXd::Zero(6);
  states.tail(3) = vel_k;

  states = update_step * states;

  return states.head(3);
}


Eigen::Affine3d MotionModel::predictPose(Eigen::Affine3d T_prev, Eigen::Affine3d T_cur, double dt)
{
  Eigen::Affine3d pose_relative = calculateRelativePose(T_prev, T_cur);
  double scale = pose_relative.translation().norm();
  double yaw = T_prev.linear().eulerAngles(0,1,2).coeff(2);

  Eigen::Affine3d pose_predicted;
  pose_predicted = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  pose_predicted.translation().x() = scale;

  return pose_predicted;
}
