#pragma once

// REF: http://docs.ros.org/en/melodic/api/gtsam/html/ISAM2__SmartFactorStereo__IMU_8cpp_source.html

/*** ROS packages ***/
#include <sensor_msgs/Imu.h>
#include <tf2_eigen/tf2_eigen.h>

#include <boost/make_shared.hpp>

/*** GTSAM packages ***/
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 

/*** Eigen packages ***/
// #include <Eigen/Geometry> 
// #include <Eigen/Dense>


class IMUPosePredictor
{
private:
  ros::Subscriber sub_; 

  double dt_;
  bool is_updated_;

  gtsam::noiseModel::Robust::shared_ptr velocity_noise_model_;
  gtsam::noiseModel::Robust::shared_ptr bias_noise_model_;
  gtsam::NavState prev_state_;
  gtsam::NavState pred_state_;
  gtsam::imuBias::ConstantBias prev_bias_;
  gtsam::PreintegratedCombinedMeasurements* preintegrated_;

  Eigen::Matrix4d T_bc_, T_cb_;   // Transformation between body and camera 

public:
  IMUPosePredictor(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size 
  ) : dt_(1 / 100.0), 
      is_updated_(false),
      sub_(nh.subscribe(topic, queue_size, &IMUPosePredictor::callback, this) )
  { 
    // Noise
    {
      gtsam::noiseModel::Diagonal::shared_ptr gaussian = gtsam::noiseModel::Isotropic::Sigma(6, 0.15);
      bias_noise_model_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);
    }

    {
      gtsam::noiseModel::Isotropic::shared_ptr gaussian = gtsam::noiseModel::Isotropic::Sigma(3, 0.3);
      velocity_noise_model_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);
    }
    // Create preintegrated instance that follows the NED frame
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81); // NED
    params->accelerometerCovariance = gtsam::I_3x3 * 0.004;  // acc white noise in continuous
    params->integrationCovariance   = gtsam::I_3x3 * 0.002;  // integration uncertainty continuous
    params->gyroscopeCovariance     = gtsam::I_3x3 * 0.001;  // gyro white noise in continuous
    params->biasAccCovariance       = gtsam::I_3x3 * 0.004;  // acc bias in continuous
    params->biasOmegaCovariance     = gtsam::I_3x3 * 0.001;  // gyro bias in continuous
    params->biasAccOmegaInt         = gtsam::Matrix::Identity(6, 6) * 1e-4;

    gtsam::Vector3 acc_bias(0.2, -0.2, -0.04);  // in camera frame
    gtsam::Vector3 gyro_bias(0.005, 0.0006, 0.024);
    gtsam::imuBias::ConstantBias prior_imu_bias = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);

    // body to IMU: rotation, translation [meters]
    gtsam::Rot3 R_bi = gtsam::Rot3(gtsam::I_3x3);
    gtsam::Point3 t_bi(0.0, 0.0, 0.0);
    params->body_P_sensor = gtsam::Pose3(R_bi, t_bi);

    gtsam::Pose3 prior_pose = gtsam::Pose3::identity();
    gtsam::Vector3 prior_velocity = gtsam::Vector3(0, 0, 0);

    prev_state_ = gtsam::NavState(prior_pose, prior_velocity);
    pred_state_ = prev_state_;
    prev_bias_ = prior_imu_bias;

    preintegrated_ = new gtsam::PreintegratedCombinedMeasurements(params, prior_imu_bias);

    T_bc_ << 1, 0, 0, 0,
             0, 0, 1, 0,
             0, 1, 0, 0,
             0, 0, 0, 1;
            
    T_cb_ = T_bc_.transpose();
  }

  ~IMUPosePredictor() {delete preintegrated_;} 


  bool isUpdated() {return is_updated_;}
  void callback(const sensor_msgs::ImuConstPtr& msg);
  Eigen::Affine3d predict();
};


void IMUPosePredictor::callback(const sensor_msgs::ImuConstPtr& msg)
{
  Eigen::Vector3d gyr, acc;
  tf2::fromMsg(msg->angular_velocity, gyr);
  tf2::fromMsg(msg->linear_acceleration, acc);
  
  preintegrated_->integrateMeasurement(acc, gyr, dt_);
  is_updated_ = true;
}



Eigen::Affine3d IMUPosePredictor::predict()
{
  pred_state_ = preintegrated_->predict(prev_state_, prev_bias_);

  prev_state_ = gtsam::NavState(pred_state_.pose(), pred_state_.velocity());
  preintegrated_->resetIntegrationAndSetBias(prev_bias_);

  is_updated_ = false;

  // return Eigen::Affine3d{pred_state_.pose().matrix()};
  return Eigen::Affine3d{T_cb_ * pred_state_.pose().matrix() * T_bc_};
}