#pragma once

/*** ROS packages ***/
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>

/*** GTSAM packages ***/
#include <gtsam/slam/PriorFactor.h> 
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 
#include <gtsam/navigation/GPSFactor.h>


class GNSSHandler
{
private:
  gtsam::noiseModel::Diagonal::shared_ptr NOISE_;

public:
  GNSSHandler() 
  {
    gtsam::Vector6 sigmas;
    sigmas << gtsam::Vector3(0.1, 0.1, 1.0), gtsam::Vector3(0.15, 0.15, 0.1);
    NOISE_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  };
  ~GNSSHandler() {};

  void addPoseFactor(int pose_id,
                     const tf2_msgs::TFMessage& msg, 
                     gtsam::Values& initial_estimate,
                     gtsam::NonlinearFactorGraph& graph);
  
  void addGNSSFactor(int pose_id,
                     const sensor_msgs::NavSatFix& msg, 
                     gtsam::Values& initial_estimate,
                     gtsam::NonlinearFactorGraph& graph);
};


void GNSSHandler::addPoseFactor(int pose_id, 
                                const tf2_msgs::TFMessage& msg,
                                gtsam::Values& initial_estimate,
                                gtsam::NonlinearFactorGraph& graph)
{
  gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(msg.transforms[0].transform.rotation.x,
                                      msg.transforms[0].transform.rotation.y,
                                      msg.transforms[0].transform.rotation.z,
                                      msg.transforms[0].transform.rotation.w);

  gtsam::Point3 translation = gtsam::Point3(msg.transforms[0].transform.translation.x,
                               msg.transforms[0].transform.translation.y,
                               msg.transforms[0].transform.translation.z);

  gtsam::Pose3 pose = gtsam::Pose3(rotation, translation);

  if (! initial_estimate.exists(gtsam::symbol_shorthand::X(pose_id)))
    initial_estimate.insert(gtsam::symbol_shorthand::X(pose_id), pose);

  graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id), 
                                             pose, 
                                             NOISE_));
}


void GNSSHandler::addGNSSFactor(int pose_id, 
                                const sensor_msgs::NavSatFix& msg,
                                gtsam::Values& initial_estimate,
                                gtsam::NonlinearFactorGraph& graph)
{
  gtsam::Point3 position = gtsam::Point3(msg.latitude,   // N,
                                         msg.longitude,  // E,
                                         msg.altitude);  // D,
  
  if (! initial_estimate.exists(gtsam::symbol_shorthand::X(pose_id)))
    initial_estimate.insert(gtsam::symbol_shorthand::X(pose_id), position);

  gtsam::GPSFactor gps_factor(gtsam::symbol_shorthand::X(pose_id),
                              position,
                              NOISE_);
  graph.add(gps_factor);
}