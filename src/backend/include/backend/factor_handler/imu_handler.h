#pragma once

// REF: http://docs.ros.org/en/melodic/api/gtsam/html/ISAM2__SmartFactorStereo__IMU_8cpp_source.html

/*** ROS packages ***/
#include <sensor_msgs/Imu.h>
#include <tf2_eigen/tf2_eigen.h>

#include <boost/make_shared.hpp>

#include <Eigen/Dense>

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
  bool initialized_;

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
    std::shared_ptr<Backend> backend,
    boost::property_tree::ptree parameters = boost::property_tree::ptree()
  ) 
  : FactorHandler(nh, topic, queue_size, backend)
  , noise_( 
      gtsam::noiseModel::Diagonal::Sigmas( 
        (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.15, 0.15, 0.1).finished()  // rad,rad,rad,m, m, m 
      ) 
    )
  , dt_( parameters.get< double >("imu.dt") )
  , from_id_(0)
  , initialized_(false)
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
    if (parameters != boost::property_tree::ptree())
    {
      Eigen::Quaterniond q(parameters.get< double >("pose.orientation.w"), 
                           parameters.get< double >("pose.orientation.x"), 
                           parameters.get< double >("pose.orientation.y"), 
                           parameters.get< double >("pose.orientation.z"));
      
      Eigen::Vector3d t(parameters.get< double >("pose.translation.x"), 
                        parameters.get< double >("pose.translation.y"), 
                        parameters.get< double >("pose.translation.z"));

      prior_pose = gtsam::Pose3(gtsam::Rot3(q), gtsam::Point3(t));

      prior_velocity = gtsam::Vector3(parameters.get< double >("imu.velocity.x"), 
                                      parameters.get< double >("imu.velocity.y"), 
                                      parameters.get< double >("imu.velocity.z"));
    }

    prev_state_ = gtsam::NavState(prior_pose, prior_velocity);
    pred_state_ = prev_state_;
    prev_bias_ = prior_imu_bias;

    preintegrated_ = new gtsam::PreintegratedCombinedMeasurements(params, prior_imu_bias);
  }

  ~IMUHandler() {delete preintegrated_;} 


  void callback(const sensor_msgs::ImuConstPtr& msg)
  {
    // ROS_INFO_STREAM("IMU measurement STAMP: " << msg->header.stamp);

    // Wait to begin preintegration until graph is initialized by another module 
    if (backend_->checkInitialized() == false)
      return;

    // Initialize by inserting initial priors to the graph
    if (backend_->checkInitialized() == true && (initialized_ == false) ) 
    {
      from_id_ = backend_->getPoseID();
      ROS_INFO_STREAM("IMU - from_id: " << from_id_);

      backend_->tryInsertValue(gtsam::symbol_shorthand::B(from_id_), prev_bias_);
      backend_->tryInsertValue(gtsam::symbol_shorthand::V(from_id_), pred_state_.velocity());

      backend_->addFactor(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(from_id_), prev_bias_, bias_noise_model_));
      backend_->addFactor(gtsam::PriorFactor<gtsam::Vector3>(gtsam::symbol_shorthand::V(from_id_), pred_state_.velocity(), velocity_noise_model_));

      initialized_ = true;
    }
    
    // Read measurements and preintegrate
    Eigen::Vector3d gyr, acc;
    tf2::fromMsg(msg->angular_velocity, gyr);
    tf2::fromMsg(msg->linear_acceleration, acc);
    
    gyr *= -1;
    acc *= -1;

    preintegrated_->integrateMeasurement(acc, gyr, dt_);

    // New pose is added --> add preintegrated measurement
    int to_id = backend_->getPoseID();
    if (to_id > from_id_)
    {
      ROS_INFO_STREAM("IMU - from_id: " << from_id_ << ", to_id: " << to_id);

      addPreintegratedFactor(to_id);
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
    
    // preintegrated_->print("PREINTEGRATION");

    // backend_->getPoseAt(pose_key_to).print("POSE AT");
    // pred_state_.pose().print("IMU PREDICT");

    gtsam::CombinedImuFactor imu_factor(pose_key_from, 
                                        vel_key_from,
                                        pose_key_to, 
                                        vel_key_to, 
                                        bias_key_from,
                                        bias_key_to, 
                                        *preintegrated_);

    backend_->addFactor(imu_factor);

    // backend_->getPoseAt(pose_key_to).print("POSE AT");
    // backend_->getPose().print("POSE MOST RECENT");

    // ROS_INFO_STREAM("backend_->getVelocity(): " << backend_->getVelocity());
    // ROS_INFO_STREAM("backend_->getValues().velocity: " << backend_->getValues().at<gtsam::Vector3>(vel_key_to));

    // Update states
    gtsam::Vector3 velocity = backend_->getVelocity();
    if ( velocity == gtsam::Vector3(0.0, 0.0, 0.0) )
      velocity = backend_->getValues().at<gtsam::Vector3>(vel_key_to);

    prev_state_ = gtsam::NavState(backend_->getPose(),
                                  velocity);

    // prev_state_ = gtsam::NavState(backend_->getValues().at<gtsam::Pose3>(pose_key_to),
    //                               backend_->getValues().at<gtsam::Vector3>(vel_key_to));
    prev_bias_ = backend_->getValues().at<gtsam::imuBias::ConstantBias>(bias_key_to);

    // ROS_INFO_STREAM("updated IMU");

    preintegrated_->resetIntegrationAndSetBias(prev_bias_);

    from_id_ = to_id;
  }
};


} // namespace factor_handler

} // namespace backend


