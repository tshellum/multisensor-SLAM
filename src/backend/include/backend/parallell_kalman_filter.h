#pragma once 

#include <gtsam/geometry/Pose3.h>
#include <gtsam_unstable/dynamics/PoseRTV.h>


class ParallellKalmanFilter
{
private: 
  gtsam::Matrix99 Q_; 
  gtsam::Matrix99 R_; 

  gtsam::PoseRTV f(const gtsam::PoseRTV& prior, double timestep)
  {
    gtsam::Vector9 x = gtsam::PoseRTV::Logmap(prior); 
    gtsam::Vector9 change = gtsam::Vector9::Zero(); 
    change(3) = timestep * x(6); 
    change(4) = timestep * x(7); 
    change(5) = timestep * x(8); 

    return gtsam::PoseRTV::Expmap(
      x + change
    ); 
  }

  gtsam::Matrix99 F(const gtsam::PoseRTV& prior, double timestep)
  {
    gtsam::Matrix99 F_boy = gtsam::Matrix99::Zero(); 
    for (int i = 0; i < 9; ++i)
      F_boy(i, i) = 1; 

    F_boy(3, 6) = timestep; 
    F_boy(4, 7) = timestep; 
    F_boy(5, 8) = timestep; 

    return F_boy; 
  }

  gtsam::Pose3 h(const gtsam::PoseRTV& predicted)
  {
    return gtsam::Pose3(
      predicted.rotation(), 
      predicted.translation()
    ); 
  }

  gtsam::Pose3 v(const gtsam::Pose3& measurement, const gtsam::Pose3& predicted_measurement)
  {
    return gtsam::Pose3::Expmap(
      gtsam::Pose3::Logmap(measurement) - gtsam::Pose3::Logmap(predicted_measurement)
    ); 
  }

  gtsam::Matrix H()
  {
    gtsam::Matrix H_boy = gtsam::Matrix::Zero(6, 9); 
    for (int i = 0; i < 6; ++i)
      H_boy(i, i) = 1; 

    return H_boy; 
  }


public: 
  ParallellKalmanFilter(gtsam::Matrix99 Q, gtsam::Matrix99 R) : Q_(Q), R_(R) {} 

  ~ParallellKalmanFilter() = default; 

  void predict(const gtsam::PoseRTV& prior, const gtsam::Matrix99& prior_covariance, double timestep, gtsam::PoseRTV& predicted, gtsam::Matrix99& predicted_covariance)
  {
    predicted = f(prior, timestep); 
    gtsam::Matrix99 predicted_jacobian = F(prior, timestep); 

    predicted_covariance = predicted_jacobian * prior_covariance * predicted_jacobian.transpose() + Q_; 
  }

  

  void update(const gtsam::Pose3& measurement, const gtsam::PoseRTV& predicted, const gtsam::Matrix99& predicted_covariance, gtsam::PoseRTV& posterior, gtsam::Matrix99& posterior_covariance)
  {
    gtsam::Pose3 predicted_measurement = h(predicted); 

    gtsam::Pose3 innovation = v(measurement, predicted_measurement);

    gtsam::Matrix measurement_jacobian = H(); 

    gtsam::Matrix99 innovation_covariance = measurement_jacobian * predicted_covariance * measurement_jacobian.transpose() + R_; 

    gtsam::Matrix kalman_gain = predicted_covariance * measurement_jacobian.transpose() * innovation_covariance.inverse(); 

    posterior = gtsam::PoseRTV::Expmap(
      gtsam::PoseRTV::Logmap(predicted) + kalman_gain * gtsam::Pose3::Logmap(innovation)
    ); 

    posterior_covariance = (gtsam::Matrix99::Identity() - kalman_gain * measurement_jacobian) * predicted_covariance; 
  }
}; 
