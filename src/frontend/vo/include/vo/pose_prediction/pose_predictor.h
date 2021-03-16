#pragma once

#include <ros/ros.h> 

#include <memory> 

#include <Eigen/Geometry> 
#include <Eigen/Dense>

#include "vo/pose_prediction/imu_preintegrator.h"
#include "vo/pose_prediction/motion_model.h"

class PosePredictor
{
private:
  MotionModel      model_;
  IMUPosePredictor imu_; 

  Eigen::Vector3d pos_k_;
  Eigen::Vector3d vel_k_;
  Eigen::Vector3d acc_k_;
  Eigen::Vector3d rot_k_;
  Eigen::Vector3d ang_k_;
  Eigen::Vector3d ang_rate_k_;

  Eigen::Affine3d T_r_pred_;

public:
  PosePredictor(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size 
  ) : imu_(nh, topic, queue_size),
      vel_k_(Eigen::Vector3d::Zero()),
      ang_k_(Eigen::Vector3d::Zero())
  {};

  ~PosePredictor() {};

  void approximateDerivative(double dt, Eigen::Affine3d T_r);

  Eigen::Affine3d predict(double dt);
};



void PosePredictor::approximateDerivative(double dt, Eigen::Affine3d T_r)
{
  Eigen::Vector3d angle = T_r.linear().eulerAngles(0,1,2); 
  ang_rate_k_ = angle / dt;

  vel_k_ = T_r.translation() / dt;
}  



Eigen::Affine3d PosePredictor::predict(double dt)
{
  if (imu_.isUpdated())
  {
    T_r_pred_   = imu_.predict();
    vel_k_      = imu_.getVelocity();
    ang_rate_k_ = imu_.getAngularRate();
  }
  else
  {
    approximateDerivative(dt, T_r_pred_);
    T_r_pred_.translation() = model_.predictTranslation(dt, ang_k_, vel_k_);
    T_r_pred_.linear()      = model_.predictRotation(dt, T_r_pred_.linear(), ang_rate_k_);
  }

  ROS_INFO_STREAM("T_r_pred_: \n" << T_r_pred_.matrix());
  // ROS_INFO_STREAM("Angles: \n" << T_r_pred_.linear().eulerAngles(0,1,2);
  // ROS_INFO_STREAM("Translation: \n" << T_r_pred_.translation();

  return T_r_pred_;
}