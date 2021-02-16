#pragma once

#include <sensor_msgs/Imu.h>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 

class IMUHandler
{
private:
  double _dt;

  gtsam::imuBias::ConstantBias _prior_imu_bias;  // assume zero initial bias
  gtsam::noiseModel::Robust::shared_ptr _velocity_noise_model;
  gtsam::noiseModel::Robust::shared_ptr _bias_noise_model;
  gtsam::NavState _prev_state;
  gtsam::NavState _pred_state;
  gtsam::imuBias::ConstantBias _prev_bias;
  gtsam::PreintegratedCombinedMeasurements* _preintegrated;
public:
  IMUHandler() : _dt(0.01)
  {    
    gtsam::noiseModel::Diagonal::shared_ptr bias_sigmas = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3(0.15, 0.15, 0.15), gtsam::Vector3(0.15, 0.15, 0.15)).finished()
    );
    _bias_noise_model = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345), bias_sigmas
    );

    gtsam::noiseModel::Isotropic::shared_ptr vel_sigmas = gtsam::noiseModel::Isotropic::Sigma(3, 0.3);
    _velocity_noise_model = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), vel_sigmas
    );

    // expect IMU to be rotated in image space co-ords
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> p = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
        gtsam::Vector3(0.0, 0.0, 9.81)
    );

    p->accelerometerCovariance = gtsam::I_3x3 * 0.004;  // acc white noise in continuous
    p->integrationCovariance   = gtsam::I_3x3 * 0.002;  // integration uncertainty continuous
    p->gyroscopeCovariance     = gtsam::I_3x3 * 0.001;  // gyro white noise in continuous
    p->biasAccCovariance       = gtsam::I_3x3 * 0.004;  // acc bias in continuous
    p->biasOmegaCovariance     = gtsam::I_3x3 * 0.001;  // gyro bias in continuous
    p->biasAccOmegaInt         = gtsam::Matrix::Identity(6, 6) * 1e-4;

    
    gtsam::Rot3 pitch = gtsam::Rot3::Pitch(0.0);
    gtsam::Rot3 roll  = gtsam::Rot3::Roll(0.0);

    // body to IMU rotation
    gtsam::Rot3 iRb = pitch * roll;

    // body to IMU translation (meters)
    gtsam::Point3 iTb(0, 0, 3);

    // body in this example is the left camera
    p->body_P_sensor = gtsam::Pose3(iRb, iTb);

    gtsam::Rot3 prior_rotation = gtsam::Rot3(gtsam::I_3x3);
    gtsam::Pose3 prior_pose(prior_rotation, gtsam::Point3(0, 0, 0));

    gtsam::Vector3 acc_bias(0.2, -0.2, -0.04);  // in camera frame
    gtsam::Vector3 gyro_bias(0.005, 0.0006, 0.024);

    _prior_imu_bias = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);

    _prev_state = gtsam::NavState(prior_pose, gtsam::Vector3(0, 0, 0));
    _pred_state = _prev_state;
    _prev_bias = _prior_imu_bias;

    _preintegrated = new gtsam::PreintegratedCombinedMeasurements(p, _prior_imu_bias);


  };

  ~IMUHandler() {};

  void readMeasurement(const sensor_msgs::ImuConstPtr& imu_msg);
  void add2graph(int& pose_id, gtsam::Values& initial_estimate, gtsam::Values& current_estimate, gtsam::NonlinearFactorGraph& graph);
};


void IMUHandler::readMeasurement(const sensor_msgs::ImuConstPtr& imu_msg){
  gtsam::Vector3 gyr(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
  gtsam::Vector3 acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);

  _preintegrated->integrateMeasurement(acc, gyr, _dt);
}


void IMUHandler::add2graph(int& pose_id, gtsam::Values& initial_estimate, gtsam::Values& current_estimate, gtsam::NonlinearFactorGraph& graph)
{
  if (pose_id == 0)
  {
    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(0), 
                                                              _prior_imu_bias,
                                                              _bias_noise_model)
    );
    
    initial_estimate.insert(gtsam::symbol_shorthand::B(0), 
                          _prior_imu_bias
    );

    graph.add(gtsam::PriorFactor<gtsam::Vector3>(gtsam::symbol_shorthand::V(0), 
                                                gtsam::Vector3(0, 0, 0), 
                                                _velocity_noise_model)
    );

    initial_estimate.insert(gtsam::symbol_shorthand::V(0), 
                            gtsam::Vector3(0, 0, 0)
    );
  }

  _pred_state = _preintegrated->predict(_prev_state, _prev_bias);

  //Legger inn initial estimates fordi det maa man ha
  initial_estimate.insert(gtsam::symbol_shorthand::X(pose_id), _pred_state.pose());
  initial_estimate.insert(gtsam::symbol_shorthand::V(pose_id), _pred_state.velocity());
  initial_estimate.insert(gtsam::symbol_shorthand::B(pose_id), _prev_bias);

  //Her legger vi inn info fra imu i grafen
  gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(pose_id - 1), 
                                     gtsam::symbol_shorthand::V(pose_id - 1),
                                     gtsam::symbol_shorthand::X(pose_id), 
                                     gtsam::symbol_shorthand::V(pose_id), 
                                     gtsam::symbol_shorthand::B(pose_id - 1),
                                     gtsam::symbol_shorthand::B(pose_id), 
                                     *_preintegrated);

  //Oppdaterer states
  _prev_state = gtsam::NavState(current_estimate.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id)),
                               current_estimate.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(pose_id))
  );
  _prev_bias = current_estimate.at<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(pose_id));
  _preintegrated->resetIntegrationAndSetBias(_prev_bias);
}
