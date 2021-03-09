#pragma once

#include <ros/ros.h> 

#include <memory> 

// #include <Eigen/Geometry> 
// #include <Eigen/Dense>

#include "vo/pose_prediction/imu_preintegrator.h"


class PosePredictor
{
private:
  std::shared_ptr<IMUPosePredictor> imu_; 
    
public:
  PosePredictor(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size 
  ) : imu_( std::make_shared<IMUPosePredictor>(nh, topic, queue_size) )
  {};

  ~PosePredictor() {};

  Eigen::Affine3d predict();

};


Eigen::Affine3d PosePredictor::predict()
{
  if (imu_->isUpdated())
    return imu_->predict();
  else
    return Eigen::Affine3d::Identity();
}