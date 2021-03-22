#pragma once

#include <ros/ros.h> 

#include <memory> 

#include <Eigen/Geometry> 
#include <Eigen/Dense>

#include "vo/pose_prediction/imu_preintegrator.h"
#include "vo/pose_prediction/motion_model.h"
#include "vo/pose_prediction/feature_based_pose_estimator.h"

class PosePredictor
{
private:
  MotionModel            model_;
  IMUPosePredictor       imu_; 
  PosePredictionFeatures feature_estimator_; 

  Eigen::Vector3d pos_k_;
  Eigen::Vector3d vel_k_;
  Eigen::Vector3d acc_k_;
  Eigen::Vector3d rot_k_;
  Eigen::Vector3d ang_k_;
  Eigen::Vector3d ang_rate_k_;

  Eigen::Affine3d T_r_pred_;

public:
  PosePredictor(){}
  
  PosePredictor(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size 
  ) : imu_(nh, topic, queue_size),
      vel_k_(Eigen::Vector3d::Zero()),
      ang_k_(Eigen::Vector3d::Zero())
  {};

  ~PosePredictor() {};

  Eigen::Affine3d getPoseRelative() {return T_r_pred_;};

  void approximateDerivative(double dt, Eigen::Affine3d T_r);

  Eigen::Affine3d predict(double dt);

  Eigen::Affine3d estimatePoseFromFeatures(std::vector<cv::KeyPoint>& kpts_prev, std::vector<cv::KeyPoint>& kpts_cur, cv::Mat K);
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






Eigen::Affine3d PosePredictor::estimatePoseFromFeatures(std::vector<cv::KeyPoint>& points_prev, std::vector<cv::KeyPoint>& points_cur, cv::Mat K)
{
  T_r_pred_ = feature_estimator_.estimatePoseFromFeatures(points_prev, points_cur, K);
  return T_r_pred_;
}
