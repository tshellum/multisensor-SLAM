#pragma once

// REF: http://docs.ros.org/en/melodic/api/gtsam/html/ISAM2__SmartFactorStereo__IMU_8cpp_source.html

/*** ROS packages ***/
#include <sensor_msgs/Imu.h>
#include <tf2_eigen/tf2_eigen.h>

#include <boost/make_shared.hpp>

/*** GTSAM packages ***/
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 



namespace backend
{

namespace factor_handler
{

class IMUHandler : public FactorHandler<const sensor_msgs::ImuConstPtr&> 
{
private:
  const gtsam::noiseModel::Diagonal::shared_ptr noise_; 
  double dt_;
  int from_id_;

  gtsam::noiseModel::Robust::shared_ptr velocity_noise_model_;
  gtsam::noiseModel::Robust::shared_ptr bias_noise_model_;
  gtsam::NavState prev_state_;
  gtsam::NavState pred_state_;
  gtsam::imuBias::ConstantBias prev_bias_;
  gtsam::PreintegratedCombinedMeasurements* preintegrated_;

  struct IMUMeasurement
  {
    gtsam::Vector3 gyro;
    gtsam::Vector3 accelerometer;

    IMUMeasurement(Eigen::Vector3d gyr, Eigen::Vector3d acc)
    : gyro( gtsam::Vector3(gyr) ), accelerometer( gtsam::Vector3(acc) )
    {}
  };

  std::map<ros::Time, IMUMeasurement> stamped_measurements_;

public:
  IMUHandler(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size, 
    std::shared_ptr<Backend> backend
  ) : FactorHandler(nh, topic, queue_size, backend), 
      noise_( 
        gtsam::noiseModel::Diagonal::Sigmas( 
          (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.15, 0.15, 0.1).finished()  // rad,rad,rad,m, m, m 
        ) 
      ),
      dt_(1 / 100.0), from_id_(0)
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

    // Insert initial values in graph
    backend->tryInsertValue(gtsam::symbol_shorthand::B(from_id_), prior_imu_bias);
    backend->tryInsertValue(gtsam::symbol_shorthand::V(from_id_), prior_velocity);

    backend_->addFactor(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(from_id_), prior_imu_bias, bias_noise_model_));
    backend_->addFactor(gtsam::PriorFactor<gtsam::Vector3>(gtsam::symbol_shorthand::V(from_id_), prior_velocity, velocity_noise_model_));
  }

  ~IMUHandler() {delete preintegrated_;} 


  void callback(const sensor_msgs::ImuConstPtr& msg)
  {
    if (backend_->checkIMUStatus().first == false)
      backend_->updateIMUOnlineStatus(true);

    Eigen::Vector3d gyr, acc;
    tf2::fromMsg(msg->angular_velocity, gyr);
    tf2::fromMsg(msg->linear_acceleration, acc);
    
    preintegrated_->integrateMeasurement(acc, gyr, dt_);
    if (backend_->getPoseID() > from_id_)
    {
      // ROS_INFO_STREAM("imu - id: " << backend_->getPoseID());
      addPreintegratedFactor(backend_->getPoseID());
      backend_->updatePreintegrationStatus(true);
    }
  }



  void addPreintegratedFactor(int to_id)
  {
    pred_state_ = preintegrated_->predict(prev_state_, prev_bias_);

    gtsam::Key pose_key_from = gtsam::symbol_shorthand::X(from_id_); 
    gtsam::Key vel_key_from  = gtsam::symbol_shorthand::V(from_id_); 
    gtsam::Key bias_key_from = gtsam::symbol_shorthand::B(from_id_); 

    gtsam::Key pose_key_to = gtsam::symbol_shorthand::X(to_id); 
    gtsam::Key vel_key_to  = gtsam::symbol_shorthand::V(to_id); 
    gtsam::Key bias_key_to = gtsam::symbol_shorthand::B(to_id); 

    backend_->tryInsertValue(pose_key_to, pred_state_.pose());
    backend_->tryInsertValue(vel_key_to, pred_state_.velocity());
    backend_->tryInsertValue(bias_key_to, prev_bias_);
      
    gtsam::CombinedImuFactor imu_factor(pose_key_from, 
                                        vel_key_from,
                                        pose_key_to, 
                                        vel_key_to, 
                                        bias_key_from,
                                        bias_key_to, 
                                        *preintegrated_);
    backend_->addFactor(imu_factor);
    
    // Update states
    prev_state_ = gtsam::NavState(backend_->getValues().at<gtsam::Pose3>(pose_key_to),
                                  backend_->getValues().at<gtsam::Vector3>(vel_key_to));
    prev_bias_ = backend_->getValues().at<gtsam::imuBias::ConstantBias>(bias_key_to);
    preintegrated_->resetIntegrationAndSetBias(prev_bias_);

    from_id_ = to_id;
  }
};


} // namespace factor_handler

} // namespace backend


