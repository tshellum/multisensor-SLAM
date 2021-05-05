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

#include <string>

#include <boost/property_tree/ptree.hpp>


namespace backend
{

namespace factor_handler
{

struct IMUMeasurement
{
  gtsam::Vector3 gyro;
  gtsam::Vector3 accelerometer;
  double dt;

  IMUMeasurement() {}

  IMUMeasurement(Eigen::Vector3d gyr, Eigen::Vector3d acc)
  : gyro( gtsam::Vector3(gyr) )
  , accelerometer( gtsam::Vector3(acc) )
  , dt( 0.01 )
  {}

  IMUMeasurement(Eigen::Vector3d gyr, Eigen::Vector3d acc, double dt)
  : gyro( gtsam::Vector3(gyr) )
  , accelerometer( gtsam::Vector3(acc) )
  , dt( dt )
  {}
};


class IMUHandler : public FactorHandler<const sensor_msgs::ImuConstPtr&> 
{
private:
  double dt_;
  ros::Time prev_stamp_;
  int from_id_;
  bool initialized_;

  gtsam::noiseModel::Robust::shared_ptr velocity_noise_model_;
  gtsam::noiseModel::Robust::shared_ptr bias_noise_model_;
  gtsam::NavState prev_state_;
  gtsam::NavState pred_state_;
  gtsam::imuBias::ConstantBias prev_bias_;
	std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> preintegrated_;

  std::map<ros::Time, IMUMeasurement> stamped_measurements_;

  gtsam::imuBias::ConstantBias initial_bias_;
  gtsam::Pose3   pose_prior_;
  gtsam::Vector3 velocity_prior_;

public:
  IMUHandler(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size, 
    std::shared_ptr<Backend> backend,
    boost::property_tree::ptree parameters = boost::property_tree::ptree()
  ) 
  : FactorHandler(nh, topic, queue_size, backend)
  , dt_( parameters.get< double >("imu.dt_avg") )
  , prev_stamp_(ros::Time(0,0))
  , from_id_(backend_->getPoseID())
  , initialized_(false)
  // , pose_prior_(gtsam::Pose3::identity())
  { 
    // Noise
    {
      // gtsam::noiseModel::Diagonal::shared_ptr gaussian = gtsam::noiseModel::Isotropic::Sigma(6, 0.15);
      bias_noise_model_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), 
        gtsam::noiseModel::Isotropic::Sigma(6, 0.15)
      );
    }

    {
      // gtsam::noiseModel::Isotropic::shared_ptr gaussian = gtsam::noiseModel::Isotropic::Sigma(3, 0.3);
      velocity_noise_model_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), 
        gtsam::noiseModel::Isotropic::Sigma(3, 0.3)
      );
    }

    // Create preintegrated instance that follows the NED frame
    std::string dataset;
	  nh.getParam("/dataset", dataset);
	
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params;
    if (parameters.get< std::string >("imu.frame") == "ENU")
      params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(); // ENU
    else if (parameters.get< std::string >("imu.frame") == "NED")
      params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(); // NED
    else
      params = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(gtsam::Vector3(0.0, 0.0, 9.81));


    /// Continuous-time "Covariance" of accelerometer
    /// The units for stddev are σ = m/s²/√Hz
    params->accelerometerCovariance = gtsam::I_3x3 * pow(parameters.get< double >("imu.uncertainty.accelerometer_sigma"), 2);       
    /// Continuous-time "Covariance" of gyroscope measurements 
    /// The units for stddev are σ = rad/s/√Hz
    params->gyroscopeCovariance = gtsam::I_3x3 * pow(parameters.get< double >("imu.uncertainty.gyroscope_sigma"), 2);  
    /// Continuous-time "Covariance" of accelerometer
    /// The units for stddev are σ = m/s²/√Hz
    params->integrationCovariance = gtsam::I_3x3 * pow(parameters.get< double >("imu.uncertainty.integration_sigma"), 2);   
    ///< continuous-time "Covariance" describing accelerometer bias random walk
    params->biasAccCovariance = gtsam::I_3x3 * pow(parameters.get< double >("imu.uncertainty.accelerometer_bias_sigma"), 2);  
    ///< continuous-time "Covariance" describing gyroscope bias random walk
    params->biasOmegaCovariance = gtsam::I_3x3 * pow(parameters.get< double >("imu.uncertainty.gyroscope_bias_sigma"), 2);
    ///< covariance of bias used for pre-integration
    params->biasAccOmegaInt = gtsam::Matrix::Identity(6, 6) * pow(parameters.get< double >("imu.uncertainty.preintegration_bias"), 2);


    // pose_prior_ = gtsam::Pose3::identity(); // To be read later
    // velocity_prior_ = gtsam::Vector3(0, 0, 0);
    if (parameters != boost::property_tree::ptree())
    {
      // velocity_prior_ = gtsam::Vector3(parameters.get< double >("imu.priors.velocity.x"), 
      //                                  parameters.get< double >("imu.priors.velocity.y"), 
      //                                  parameters.get< double >("imu.priors.velocity.z"));

      double t_params[3]; 
      double R_params[9]; 

      int i = 0;
      BOOST_FOREACH(boost::property_tree::ptree::value_type &v, parameters.get_child("imu.bodyTimu.rotation"))
        R_params[i++] = v.second.get_value<double>(); 

      i = 0;
      BOOST_FOREACH(boost::property_tree::ptree::value_type &v, parameters.get_child("imu.bodyTimu.translation"))
        t_params[i++] = v.second.get_value<double>(); 
    
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > R(R_params);
      Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor> > t(t_params);
      
      Eigen::Affine3d bodyTimu_eigen = Eigen::Affine3d::Identity();
      bodyTimu_eigen.linear() = R;
      bodyTimu_eigen.translation() = t;

      params->body_P_sensor = gtsam::Pose3(bodyTimu_eigen.matrix());
    }
    
    gtsam::Vector3 acc_bias(parameters.get< double >("imu.priors.accelerometer.x"), 
                            parameters.get< double >("imu.priors.accelerometer.y"), 
                            parameters.get< double >("imu.priors.accelerometer.z"));  
    gtsam::Vector3 gyro_bias(parameters.get< double >("imu.priors.gyroscope.roll_rate"), 
                             parameters.get< double >("imu.priors.gyroscope.pitch_rate"), 
                             parameters.get< double >("imu.priors.gyroscope.yaw_rate"));

    // gtsam::imuBias::ConstantBias prior_imu_bias = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);
    initial_bias_ = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);
    prev_bias_ = initial_bias_;

    // backend_->updateVelocity(velocity_prior_);
    // backend_->updateBias(initial_bias_);

    // prev_state_ = gtsam::NavState(pose_prior_, velocity_prior_);
    // pred_state_ = prev_state_;

    preintegrated_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(params, initial_bias_);
  }

  ~IMUHandler() {} 


  void callback(const sensor_msgs::ImuConstPtr& msg)
  {
    // Wait to begin preintegration until graph is initialized by another module 
    if (! backend_->checkInitialized() )
      return;

    // Read measurements and save measurement
    Eigen::Vector3d gyr, acc;
    tf2::fromMsg(msg->angular_velocity, gyr);
    tf2::fromMsg(msg->linear_acceleration, acc);


    // Initialize by inserting initial priors to the graph
    if ( backend_->checkInitialized() && ! initialized_ ) 
    {
      from_id_ = backend_->getPoseID();
      // pose_prior_ = backend_->getPoseAt(gtsam::symbol_shorthand::X(from_id_));
      // prev_state_ = gtsam::NavState(pose_prior_, 
      //                               pose_prior_.rotation() * backend_->getVelocity());
      // pred_state_ = prev_state_;

      // backend_->tryInsertValue(gtsam::symbol_shorthand::B(from_id_), prev_bias_);
      // backend_->tryInsertValue(gtsam::symbol_shorthand::V(from_id_), pred_state_.velocity());

      // backend_->addFactor(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(from_id_), prev_bias_, bias_noise_model_));
      // backend_->addFactor(gtsam::PriorFactor<gtsam::Vector3>(gtsam::symbol_shorthand::V(from_id_), pred_state_.velocity(), velocity_noise_model_));

      initialized_ = true;

      stamped_measurements_[msg->header.stamp] = IMUMeasurement(gyr, acc, dt_);
    }
    else
      stamped_measurements_[msg->header.stamp] = IMUMeasurement(gyr, acc, (msg->header.stamp - prev_stamp_).toSec());

    // stamped_measurements_[msg->header.stamp] = IMUMeasurement(gyr, acc, dt_);


    // New pose is added --> add preintegrated measurement
    int to_id = backend_->getNewestPoseID();
    if  (to_id > from_id_)
    {        
      ros::Time from_time = backend_->findPoseStamp(from_id_);
      ros::Time to_time = backend_->getNewestPoseTime();

      // Integrate measurements and delete all measurements that was integrated
      std::map<ros::Time, IMUMeasurement>::iterator to_it = integrateMeasurements(from_time, to_time);
      stamped_measurements_.erase(stamped_measurements_.begin(), to_it);

      // Add preintegrate measurement to factor graph
      addPreintegratedFactor(from_id_, to_id);


      // Update states
      prev_state_ = gtsam::NavState(backend_->getPoseAt(gtsam::symbol_shorthand::X(to_id)),
                                    backend_->getVelocity());

      prev_bias_ = backend_->getBias();

      // Reset integration
      preintegrated_->resetIntegrationAndSetBias(prev_bias_);

      from_id_ = to_id;
    }
    
    // Update stamp of previous IMU measurement for next dt
    prev_stamp_ = msg->header.stamp;
  }



  void addPreintegratedFactor(int from_id, int to_id)
  {
    gtsam::Key pose_key_from = gtsam::symbol_shorthand::X(from_id); 
    gtsam::Key vel_key_from  = gtsam::symbol_shorthand::V(from_id); 
    gtsam::Key bias_key_from = gtsam::symbol_shorthand::B(from_id); 

    gtsam::Key pose_key_to = gtsam::symbol_shorthand::X(to_id); 
    gtsam::Key vel_key_to  = gtsam::symbol_shorthand::V(to_id); 
    gtsam::Key bias_key_to = gtsam::symbol_shorthand::B(to_id); 

    // std::cout << "\nVelocity from: " << prev_state_.velocity().transpose() << ", Norm: " << prev_state_.velocity().norm() << std::endl;
    // std::cout << "Velocity to: " << pred_state_.velocity().transpose() << ", Norm: " << pred_state_.velocity().norm() << std::endl;

    if ( (! backend_->valueExist(vel_key_from)) || (! backend_->valueExist(bias_key_from)))
      reinitializeIMU(from_id, to_id);

    // if ( backend_->tryInsertValue(vel_key_from, prev_state_.velocity()) )
    //   backend_->addFactor(gtsam::PriorFactor<gtsam::Vector3>(vel_key_from, prev_state_.velocity(), velocity_noise_model_));
    // if ( backend_->tryInsertValue(bias_key_from, prev_bias_) )
    //   backend_->addFactor(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(bias_key_from, prev_bias_, bias_noise_model_));

  
    pred_state_ = preintegrated_->predict(prev_state_, prev_bias_);

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
  }


  std::map<ros::Time, IMUMeasurement>::iterator integrateMeasurements(ros::Time from_time, ros::Time to_time)
  {
    std::map<ros::Time, IMUMeasurement>::iterator imu_stamped_measurement = stamped_measurements_.begin();
    while ( (imu_stamped_measurement->first <= to_time) && (imu_stamped_measurement != stamped_measurements_.end()))
    {
      if (imu_stamped_measurement->first > from_time)      
        preintegrated_->integrateMeasurement(imu_stamped_measurement->second.accelerometer, 
                                            imu_stamped_measurement->second.gyro, 
                                            imu_stamped_measurement->second.dt);

      imu_stamped_measurement++;
    }

    return imu_stamped_measurement;
  }

  void reinitializeIMU(int from_id, int to_id)
  {
    ros::Time from_time = backend_->findPoseStamp(from_id_);
    ros::Time to_time = backend_->getNewestPoseTime();

    gtsam::Pose3 from_pose = backend_->getPoseAt(gtsam::symbol_shorthand::X(from_id));
    gtsam::Pose3 to_pose = backend_->getPoseAt(gtsam::symbol_shorthand::X(to_id));

    gtsam::Vector3 velocity = approximateVelocityWorldFrame(from_pose, to_pose, (to_time - from_time).toSec());
    gtsam::imuBias::ConstantBias bias = backend_->getBias();

    gtsam::Key vel_key_from  = gtsam::symbol_shorthand::V(from_id); 
    gtsam::Key bias_key_from = gtsam::symbol_shorthand::B(from_id); 

    if ( backend_->tryInsertValue(vel_key_from, velocity) )
      backend_->addFactor(gtsam::PriorFactor<gtsam::Vector3>(vel_key_from, velocity, velocity_noise_model_));
    if ( backend_->tryInsertValue(bias_key_from, bias) )
      backend_->addFactor(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(bias_key_from, bias, bias_noise_model_));

    // pose_prior_ = backend_->getPoseAt(gtsam::symbol_shorthand::X(from_id_));
    // prev_state_ = gtsam::NavState(pose_prior_, 
    //                               pose_prior_.rotation() * velocity);

    gtsam::Pose3 pose_from = backend_->getPoseAt(gtsam::symbol_shorthand::X(from_id_));
    
    prev_state_ = gtsam::NavState(pose_from, 
                                  pose_from.rotation() * velocity);
    pred_state_ = prev_state_;
    prev_bias_ = initial_bias_;

    backend_->updateVelocity(velocity);
    backend_->updateBias(initial_bias_);
  }

  gtsam::Vector3 approximateVelocityWorldFrame(gtsam::Pose3 pose_previous, gtsam::Pose3 pose_current, double dt)
  {
    gtsam::Pose3 between = pose_previous.between(pose_current);
    gtsam::Vector3 velocity_body = gtsam::Vector3(between.translation().norm(), 0.0, 0.0) / dt;
    double avg_yaw = (pose_previous.rotation().yaw() + pose_current.rotation().yaw()) / 2;

    // TODO: snitt av prev og cur yaw
    // TODO: Anta bevegelse fremover (x = norm) * snitt(yaw)
    // return ( pose_previous.rotation() * between.translation() ) / dt;
    return gtsam::Rot3::Rz(avg_yaw) * velocity_body;
  }


};


} // namespace factor_handler

} // namespace backend


