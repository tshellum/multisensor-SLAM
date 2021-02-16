#pragma once

#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>

// #include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/PriorFactor.h> 
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 

class GNSSHandler
{
private:
  // double _timestamp;
  std::vector<double> _timestamps;

  gtsam::Rot3   _rotation;
  gtsam::Point3 _translation;
  gtsam::noiseModel::Diagonal::shared_ptr _NOISE;
public:
  GNSSHandler() 
  {
    gtsam::Vector6 sigmas;
    sigmas << gtsam::Vector3(0.1, 0.1, 1.0), gtsam::Vector3(0.15, 0.15, 0.1);
    _NOISE = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  };
  ~GNSSHandler() {};

  void addPose2Graph(int& pose_id, std::map<double, int>& timestamped_ids,
                     const tf2_msgs::TFMessage& msg, gtsam::NonlinearFactorGraph& graph);
};



void GNSSHandler::addPose2Graph(int& pose_id, std::map<double, int>& timestamped_ids, 
                                const tf2_msgs::TFMessage& msg, gtsam::NonlinearFactorGraph& graph)
{
  double timestamp = msg.transforms[0].header.stamp.sec + (msg.transforms[0].header.stamp.nsec / 1e9);
  timestamped_ids.insert(timestamped_ids.end(), std::pair<double,int>(timestamp, ++pose_id));

  _rotation = gtsam::Rot3::Quaternion(msg.transforms[0].transform.rotation.x,
                                      msg.transforms[0].transform.rotation.y,
                                      msg.transforms[0].transform.rotation.z,
                                      msg.transforms[0].transform.rotation.w);

  _translation = gtsam::Point3(msg.transforms[0].transform.translation.x,
                               msg.transforms[0].transform.translation.y,
                               msg.transforms[0].transform.translation.z);

  graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id), 
                                                   gtsam::Pose3(_rotation, _translation), 
                                                   _NOISE));
}


// void GNSSHandler::addPose2Graph(const sensor_msgs::NavSat& msg, gtsam::NonlinearFactorGraph& graph)
// {
//   _timestamp = msg.transforms[0].header.stamp.sec + (msg.transforms[0].header.stamp.nsec / 1e9);
  
//   graph.push_back(gtsam::GPSFactor(gtsam::symbol_shorthand::X(_pose_id++), gtsam::Point3(msg.latitude, msg.longitude, msg.altitude), _NOISE))
// }



