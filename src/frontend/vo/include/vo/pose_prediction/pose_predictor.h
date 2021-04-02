#pragma once

#include <ros/ros.h> 

#include <memory> 

#include <Eigen/Geometry> 
#include <Eigen/Dense>

#include "vo/pose_prediction/imu_preintegrator.h"
// #include "vo/pose_prediction/motion_model.h"
#include "vo/pose_prediction/feature_based_pose_estimator.h"

class PosePredictor
{
private:
  // MotionModel            model_;
  IMUPosePredictor       imu_; 
  PosePredictionFeatures feature_estimator_; 

  // For motion model
  Eigen::Vector3d pos_k_;
  Eigen::Vector3d vel_k_;
  Eigen::Vector3d acc_k_;
  Eigen::Vector3d rot_k_;
  Eigen::Vector3d ang_k_;
  Eigen::Vector3d ang_rate_k_;


  // Predicted variables
  Eigen::Affine3d T_r_pred_;

  Eigen::Matrix4d T_cb_;
  Eigen::Matrix4d T_bc_;

  double scale_;

public:
  PosePredictor(){}
  
  PosePredictor(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size 
  ) : imu_(nh, topic, queue_size)
    , vel_k_(Eigen::Vector3d::Zero())
    , ang_k_(Eigen::Vector3d::Zero())
    , scale_(1.0)

  {
    T_bc_ << 0, 0, 1, 0,
             1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 0, 1;

    T_cb_ = T_bc_.transpose();
  };

  ~PosePredictor() {};

  Eigen::Affine3d getPoseRelative() {return T_r_pred_;};
  double getPreviousScale() {return scale_;};

  void approximateDerivative(double dt, Eigen::Affine3d T_r);

  Eigen::Affine3d predict(double dt);

  Eigen::Affine3d estimatePoseFromFeatures(std::vector<cv::KeyPoint>& kpts_prev, std::vector<cv::KeyPoint>& kpts_cur, cv::Mat K);

  Eigen::Affine3d cam2body(Eigen::Affine3d T_c);

  void updatePredicted(Eigen::Affine3d T);
  double calculateScale(Eigen::Vector3d t);
  double calculateScale(Eigen::Vector3d t, double previous_scale);
  bool evaluateValidity(Eigen::Affine3d T_cur, Eigen::Affine3d T_prev, double rot_thresh = M_PI/6);
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
    // T_r_pred_.translation() = model_.predictTranslation(dt, ang_k_, vel_k_);
    // T_r_pred_.linear()      = model_.predictRotation(dt, T_r_pred_.linear(), ang_rate_k_);
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



Eigen::Affine3d PosePredictor::cam2body(Eigen::Affine3d T_clcr)
{
  return Eigen::Affine3d{T_bc_ * T_clcr.matrix() * T_cb_}; 
}



void PosePredictor::updatePredicted(Eigen::Affine3d T)
{
  T_r_pred_ = T; 
}


double PosePredictor::calculateScale(Eigen::Vector3d t)
{
  double norm = t.norm(); 
  if (norm < 10)
    scale_ = norm;
  // else: scale is unchanged / set to previous
  return scale_;
}



double PosePredictor::calculateScale(Eigen::Vector3d t, double previous_scale)
{
  double scale = t.norm(); 
  if (scale > 10) // scale is unchanged / set to previous
    scale = previous_scale;

  return scale;
}


bool PosePredictor::evaluateValidity(Eigen::Affine3d T_cur, Eigen::Affine3d T_prev, double rot_thresh)
{
  Eigen::Vector3d rot_euler_cur  = T_cur.linear().eulerAngles(0,1,2);
  Eigen::Vector3d rot_euler_prev = T_prev.linear().eulerAngles(0,1,2);
  Eigen::Vector3d rot_diff_abs = (rot_euler_cur - rot_euler_prev).cwiseAbs(); 

  // No major difference in rotational estimates since prev
  return ( (rot_diff_abs.x() < rot_thresh)  
        && (rot_diff_abs.y() < rot_thresh)  
        && (rot_diff_abs.z() < rot_thresh) 
        && (std::abs(T_cur(2,3)) > std::abs(T_cur(0,3)))        // No sideways motion  
        && (std::abs(T_cur(2,3)) > std::abs(T_cur(1,3)))        // No upwards/downwards motion  
  );
}
