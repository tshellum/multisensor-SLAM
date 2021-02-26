#pragma once

// REF: http://docs.ros.org/en/melodic/api/gtsam/html/ISAM2__SmartFactorStereo__IMU_8cpp_source.html

/*** ROS packages ***/
#include <sensor_msgs/Imu.h>
#include <tf2_eigen/tf2_eigen.h>

/*** GTSAM packages ***/
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
    double dt_;

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
          ( gtsam::Vector6() 
            << gtsam::Vector3(0.1, 0.1, 0.1), 
            gtsam::Vector3(0.15, 0.15, 0.1) 
          ).finished() 
        ) 
      ),
      dt_(1 / 100.0), from_id_(0)
  { 
    {
      gtsam::noiseModel::Diagonal::shared_ptr gaussian = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.15), gtsam::Vector3::Constant(0.15)).finished());
      bias_noise_model_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);
    }

    {
      gtsam::noiseModel::Isotropic::shared_ptr gaussian = gtsam::noiseModel::Isotropic::Sigma(3, 0.3);
      velocity_noise_model_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);
    }

    // Follows NED frame
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> p = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
        gtsam::Vector3(0.0, 0.0, 9.81)
    );

    p->accelerometerCovariance = gtsam::I_3x3 * 0.004;  // acc white noise in continuous
    p->integrationCovariance   = gtsam::I_3x3 * 0.002;  // integration uncertainty continuous
    p->gyroscopeCovariance     = gtsam::I_3x3 * 0.001;  // gyro white noise in continuous
    p->biasAccCovariance       = gtsam::I_3x3 * 0.004;  // acc bias in continuous
    p->biasOmegaCovariance     = gtsam::I_3x3 * 0.001;  // gyro bias in continuous
    p->biasAccOmegaInt         = gtsam::Matrix::Identity(6, 6) * 1e-4;


    // body to IMU rotation
    gtsam::Rot3 R_bi = gtsam::Rot3(gtsam::I_3x3);

    // body to IMU translation (meters)
    gtsam::Point3 t_bi(0.0, 0.0, 0.0);

    // body in this example is the left camera
    p->body_P_sensor = gtsam::Pose3(R_bi, t_bi);

    gtsam::Rot3 prior_rotation = gtsam::Rot3(gtsam::I_3x3);
    gtsam::Pose3 prior_pose(prior_rotation, gtsam::Point3(0, 0, 0));

    gtsam::Vector3 acc_bias(0.2, -0.2, -0.04);  // in camera frame
    gtsam::Vector3 gyro_bias(0.005, 0.0006, 0.024);

    gtsam::imuBias::ConstantBias prior_imu_bias = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);

    prev_state_ = gtsam::NavState(prior_pose, gtsam::Vector3(0, 0, 0));
    pred_state_ = prev_state_;
    prev_bias_ = prior_imu_bias;

    preintegrated_ = new gtsam::PreintegratedCombinedMeasurements(p, prior_imu_bias);
  }

  ~IMUHandler() = default; 

  void callback(const sensor_msgs::ImuConstPtr& msg)
  {
    Eigen::Vector3d gyr, acc;
    tf2::fromMsg(msg->angular_velocity, gyr);
    tf2::fromMsg(msg->linear_acceleration, acc);
    
    stamped_measurements_.insert( 
      stamped_measurements_.end(), // for increased efficiency (we know it will be added to back)
      std::pair<ros::Time, IMUMeasurement>(
        msg->header.stamp, 
        IMUMeasurement(gyr, acc)) 
    );
  }

  void addPreintegratedFactor(int to_id, ros::Time to_time)
  {
    // Preintegrate relevant measurements
    std::map<ros::Time, IMUMeasurement>::iterator stamped_measurement = stamped_measurements_.begin();
    while (stamped_measurement != stamped_measurements_.end())
    {
      if (to_time < stamped_measurement->first)
        break;

      preintegrated_->integrateMeasurement(
        stamped_measurement->second.accelerometer, 
        stamped_measurement->second.gyro, 
        dt_
      );
      stamped_measurements_.erase(stamped_measurement); 

      stamped_measurement++;;
    }

    // Add to graph
    pred_state_ = preintegrated_->predict(prev_state_, prev_bias_);

    // Legger inn initial estimates fordi det må man ha
    gtsam::Key pose_key_from = gtsam::symbol_shorthand::X(from_id_); 
    gtsam::Key vel_key_from  = gtsam::symbol_shorthand::V(from_id_); 
    gtsam::Key bias_key_from = gtsam::symbol_shorthand::B(from_id_); 

    gtsam::Key pose_key_to = gtsam::symbol_shorthand::X(to_id); 
    gtsam::Key vel_key_to  = gtsam::symbol_shorthand::V(to_id); 
    gtsam::Key bias_key_to = gtsam::symbol_shorthand::B(to_id); 

    backend_->insertValue(pose_key_to, pred_state_.pose());
    backend_->insertValue(vel_key_to, pred_state_.velocity());
    backend_->insertValue(bias_key_to, prev_bias_);
    
    // Her legger vi inn info fra imu i grafen
    gtsam::CombinedImuFactor imu_factor(pose_key_from, 
                                        vel_key_from,
                                        pose_key_to, 
                                        vel_key_to, 
                                        bias_key_from,
                                        bias_key_to, 
                                        *preintegrated_);
    backend_->getGraph().add(imu_factor);
    
    // Oppdaterer states
    prev_state_ = gtsam::NavState(backend_->getValues().at<gtsam::Pose3>(pose_key_to),
                                  backend_->getValues().at<gtsam::Vector3>(vel_key_to));
    prev_bias_ = backend_->getValues().at<gtsam::imuBias::ConstantBias>(bias_key_to);
    preintegrated_->resetIntegrationAndSetBias(prev_bias_);

    from_id_ = to_id;
  }


};


} // namespace factor_handler

} // namespace backend



// class IMUHandler
// {
// private:
//   double dt_;
//   int imu_measurement_id_;
//   int from_imu_id_;

//   std::map<ros::Time, gtsam::NavState> measurements_;

//   gtsam::noiseModel::Robust::shared_ptr velocity_noise_model_;
//   gtsam::noiseModel::Robust::shared_ptr bias_noise_model_;
//   gtsam::NavState prev_state_;
//   gtsam::NavState pred_state_;
//   gtsam::imuBias::ConstantBias prev_bias_;
//   gtsam::PreintegratedCombinedMeasurements* preintegrated_;

// public:
//   IMUHandler(int pose_id, gtsam::Values& initial_estimate, gtsam::NonlinearFactorGraph& graph) 
//   : dt_(1 / 100.0), 
//     imu_measurement_id_(0), from_imu_id_(0)
//   {    
//     gtsam::noiseModel::Diagonal::shared_ptr bias_sigmas = gtsam::noiseModel::Diagonal::Sigmas(
//       (gtsam::Vector(6) << gtsam::Vector3(0.15, 0.15, 0.15), gtsam::Vector3(0.15, 0.15, 0.15)).finished()
//     );
//     bias_noise_model_ = gtsam::noiseModel::Robust::Create(
//       gtsam::noiseModel::mEstimator::Huber::Create(1.345), bias_sigmas
//     );

//     gtsam::noiseModel::Isotropic::shared_ptr vel_sigmas = gtsam::noiseModel::Isotropic::Sigma(3, 0.3);
//     velocity_noise_model_ = gtsam::noiseModel::Robust::Create(
//         gtsam::noiseModel::mEstimator::Huber::Create(1.345), vel_sigmas
//     );

//     // expect IMU to be rotated in image space co-ords
//     boost::shared_ptr<gtsam::PreintegrationCombinedParams> p = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
//         gtsam::Vector3(0.0, 0.0, 9.81)
//     );

//     p->accelerometerCovariance = gtsam::I_3x3 * 0.004;  // acc white noise in continuous
//     p->integrationCovariance   = gtsam::I_3x3 * 0.002;  // integration uncertainty continuous
//     p->gyroscopeCovariance     = gtsam::I_3x3 * 0.001;  // gyro white noise in continuous
//     p->biasAccCovariance       = gtsam::I_3x3 * 0.004;  // acc bias in continuous
//     p->biasOmegaCovariance     = gtsam::I_3x3 * 0.001;  // gyro bias in continuous
//     p->biasAccOmegaInt         = gtsam::Matrix::Identity(6, 6) * 1e-4;


//     gtsam::Rot3 prior_rotation = gtsam::Rot3(gtsam::I_3x3);
//     gtsam::Pose3 prior_pose(prior_rotation, gtsam::Point3(0, 0, 0));

//     gtsam::Vector3 acc_bias(0.2, -0.2, -0.04);  // in camera frame
//     gtsam::Vector3 gyro_bias(0.005, 0.0006, 0.024);

//     gtsam::imuBias::ConstantBias prior_imu_bias = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);

//     prev_state_ = gtsam::NavState(prior_pose, gtsam::Vector3(0, 0, 0));
//     pred_state_ = prev_state_;
//     prev_bias_ = prior_imu_bias;

//     preintegrated_ = new gtsam::PreintegratedCombinedMeasurements(p, prior_imu_bias);
    
//     initial_estimate.insert(gtsam::symbol_shorthand::X(pose_id), gtsam::Pose3::identity());
//     initial_estimate.insert(gtsam::symbol_shorthand::B(pose_id), prior_imu_bias);
//     initial_estimate.insert(gtsam::symbol_shorthand::V(pose_id), gtsam::Vector3(0, 0, 0));

//     // Add priors
//     gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
//        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());
//     graph.addPrior(gtsam::symbol_shorthand::X(pose_id), gtsam::Pose3::identity(), prior_pose_noise);
//     graph.addPrior(gtsam::symbol_shorthand::B(pose_id), prior_imu_bias, bias_noise_model_);
//     graph.addPrior(gtsam::symbol_shorthand::V(pose_id), gtsam::Vector3(0, 0, 0), velocity_noise_model_);
//   };

//   ~IMUHandler() { delete preintegrated_; };

//   void preintegrateMeasurement(const sensor_msgs::ImuConstPtr& imu_msg);
//   void addPoseFactor(int pose_id, gtsam::Values& initial_estimate, gtsam::NonlinearFactorGraph& graph);
// };


// void IMUHandler::preintegrateMeasurement(const sensor_msgs::ImuConstPtr& imu_msg){
//   gtsam::Vector3 gyr(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
//   gtsam::Vector3 acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);

//   preintegrated_->integrateMeasurement(acc, gyr, dt_);
//   measurements_[imu_msg->header.stamp] = preintegrated_->predict(prev_state_, prev_bias_);
//   imu_measurement_id_++;
// }



// void IMUHandler::addPoseFactor(int pose_id, gtsam::Values& initial_estimate, gtsam::NonlinearFactorGraph& graph)
// {
//   if (imu_measurement_id_ > 0)
//   {
//     pred_state_ = preintegrated_->predict(prev_state_, prev_bias_);

//     // Legger inn initial estimates fordi det må man ha
//     initial_estimate.insert(gtsam::symbol_shorthand::X(pose_id), pred_state_.pose());
//     initial_estimate.insert(gtsam::symbol_shorthand::V(pose_id), pred_state_.velocity());
//     initial_estimate.insert(gtsam::symbol_shorthand::B(pose_id), prev_bias_);
    
//     // Her legger vi inn info fra imu i grafen
//     gtsam::CombinedImuFactor imu_factor(gtsam::symbol_shorthand::X(from_imu_id_), 
//                                         gtsam::symbol_shorthand::V(from_imu_id_),
//                                         gtsam::symbol_shorthand::X(pose_id), 
//                                         gtsam::symbol_shorthand::V(pose_id), 
//                                         gtsam::symbol_shorthand::B(from_imu_id_),
//                                         gtsam::symbol_shorthand::B(pose_id), 
//                                         *preintegrated_);
//     graph.add(imu_factor);

//     // Oppdaterer states
//     prev_state_ = gtsam::NavState(initial_estimate.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id)),
//                                   initial_estimate.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(pose_id)));
//     prev_bias_ = initial_estimate.at<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(pose_id));
//     preintegrated_->resetIntegrationAndSetBias(prev_bias_);

//     from_imu_id_ = pose_id;
//     imu_measurement_id_ = 0;
//   }
// }



