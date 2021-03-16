#pragma once

/*** ROS packages ***/
#include <ros/ros.h> 

#include <memory> 

/*** Eigen packages ***/
#include <Eigen/Geometry> 
#include <Eigen/Dense>

/*** Sophus packages ***/
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

/*** Local ***/
#include "vo/pose_prediction/imu_preintegrator.h"


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

  Eigen::Vector3d predictTranslation(double dt, Eigen::Vector3d angle_k, Eigen::Vector3d vel_k);
  Eigen::Matrix3d predictRotation(double dt, Eigen::Matrix3d angle_k, Eigen::Vector3d omega_k);
};


Eigen::Vector3d MotionModel::predictTranslation(double dt, Eigen::Vector3d ang_k, Eigen::Vector3d vel_k)
{
  double yaw = ang_k.coeff(2);
  Eigen::MatrixXd update_step = base_;
  update_step.block<2,2>(0,3) = Eigen::Scaling(dt) * Eigen::Rotation2Dd(yaw);

  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.tail(3) = vel_k;

  // ROS_INFO_STREAM("pose_{k}: \n" << pose);
  // ROS_INFO_STREAM("update_step: \n" << update_step);
  
  pose = update_step * pose;
  // ROS_INFO_STREAM("pose_{k+1}: \n" << pose);

  return pose.head(3);
}



// TODO: Sophus for pertubating rotation updates 
// https://www.programmersought.com/article/8815619131/
// https://vision.in.tum.de/_media/teaching/ws2020/visnav_ws2020/visnav_lecture1.pdf

Eigen::Matrix3d MotionModel::predictRotation(double dt, Eigen::Matrix3d R_k, Eigen::Vector3d omega_k)
{
  Sophus::SO3d SO3_prev(R_k);

  // Update
  // TODO: Formulate update
  Sophus::SO3d SO3_next = Sophus::SO3d::exp(Eigen::Scaling(dt) * omega_k) * SO3_prev; // Left multiplication model

  return Eigen::Matrix3d{ SO3_next.matrix() };
}
