#pragma once

/*** ROS packages ***/
#include <ros/ros.h> 
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
// #include <sensor_msgs/NavSatFix.h>
// #include "backend/NorthEastHeading.h"

/*** Eigen packages ***/
#include <Eigen/Dense>

/*** GTSAM packages ***/
#include <gtsam/slam/PriorFactor.h> 
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 

/*** Boost packages ***/
#include <boost/property_tree/ptree.hpp>

// Local 
#include "backend/factor_handler/factor_handler.h"



enum { OFFLINE = 0, ONLINE = 1 };


namespace backend
{

namespace factor_handler
{

class GNSSHandler : public FactorHandler<const tf2_msgs::TFMessage&> 
{
private:
  gtsam::noiseModel::Diagonal::shared_ptr noise_; 
  int pose_id_;
  ros::Time prev_stamp_;

  double dt_;
  std::map<ros::Time, gtsam::Pose3> pendig_gnss_measurements_;

public:
  GNSSHandler(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size, 
    std::shared_ptr<Backend> backend,
    boost::property_tree::ptree parameters = boost::property_tree::ptree()  
  ) 
  : FactorHandler(nh, topic, queue_size, backend, parameters.get< bool >("sensor_status.gnss", false))
  , pose_id_(backend_->getPoseID())
  , noise_( gtsam::noiseModel::Diagonal::Sigmas(
      ( gtsam::Vector(6) << gtsam::Vector3::Constant( parameters.get< double >("gnss.orientation_sigma", M_PI/180) ), 
                            gtsam::Vector3::Constant( parameters.get< double >("gnss.position_sigma", 0.1) )
      ).finished() )
    )
  , dt_(parameters.get< double >("dt", 1.0))
  {
    if (online_)
    {
      backend->updateNavStatus(ONLINE); 
      std::cout << "- GNSS" << std::endl;
    }
  }
  ~GNSSHandler() = default; 

  void queueMeasurement(ros::Time stamp, gtsam::Pose3 measurement) { pendig_gnss_measurements_[stamp] = measurement; }

  gtsam::Pose3 queryGNSSMeasurement(ros::Time stamp, std::map<ros::Time, gtsam::Pose3> pendig_gnss_measurements)
  {
    std::map<ros::Time, gtsam::Pose3>::iterator cur_measurement_it = pendig_gnss_measurements.begin();
    std::map<ros::Time, gtsam::Pose3>::iterator prev_measurement_it = pendig_gnss_measurements.begin();
    
    int i = 0;
    while (cur_measurement_it != pendig_gnss_measurements.end())
    {
      if (stamp < cur_measurement_it->first)
        break;
  
      cur_measurement_it++;
      if (i++ > 0)
        prev_measurement_it = pendig_gnss_measurements.erase(prev_measurement_it);
    }

    double cur_diff = std::abs( (stamp - cur_measurement_it->first).toSec() );
    double prev_diff = std::abs( (stamp - prev_measurement_it->first).toSec() );

    if ( cur_diff > prev_diff ) // Ensure that previous association isn't the actual previous pose 
      return prev_measurement_it->second;           // Previous is associated
    else
      return cur_measurement_it->second;            // Current is associated
  }


  void callback(const tf2_msgs::TFMessage& msg)
  {
    Eigen::Isometry3d T_w = tf2::transformToEigen(msg.transforms[0].transform); 
    gtsam::Pose3 pose(T_w.matrix()); 

    if ( (! backend_->checkInitialized()) || online_ )
      queueMeasurement(msg.transforms[0].header.stamp, pose);
    else
      return;

    if (! backend_->isOdometryPending())
      return;

    ros::Time stamp = backend_->newestOdometryStamp();

    gtsam::Pose3 associated_pose = queryGNSSMeasurement(stamp, pendig_gnss_measurements_);
    pose_id_ = backend_->getPoseID();

    backend_->setOdometryStamp(ros::Time(0.0));  // Set odometry to not pending

    if (! backend_->checkInitialized()) 
    {
      backend_->registerStampedPose(stamp, pose_id_);

      backend_->updatePose(associated_pose);
      backend_->setWorldOrigin(associated_pose);

      backend_->setInitialized(true);  
    }
    else
    {
      if ( (stamp - prev_stamp_).toSec() < dt_ )
        return;      
    }

    gtsam::Key pose_key = gtsam::symbol_shorthand::X(pose_id_);       
    backend_->tryInsertValue(pose_key, associated_pose);
    backend_->addFactor(
      gtsam::PriorFactor<gtsam::Pose3>(
        pose_key, associated_pose, noise_
      )
    );

    // std::cout << "GNSS callback() adding - ID: " << pose_id_ << ", stamp: " << stamp << std::endl;

    prev_stamp_ = stamp;
    backend_->isUpdated();
  }

};


} // namespace factor_handler

} // namespace backend
