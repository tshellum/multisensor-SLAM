/*** ROS packages ***/
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

/*** GTSAM packages ***/
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 

/*** Class packages ***/
#include "support.h"


class StereoHandler
{
private:
  int from_id_;
  int to_id_;
  
  gtsam::Rot3 world_rotation_;
  gtsam::noiseModel::Diagonal::shared_ptr NOISE_;

public:
  StereoHandler() : from_id_(0), to_id_(0)
  {
    gtsam::Vector6 sigmas;
    sigmas << gtsam::Vector3(1.0, 1.0, 1.0), gtsam::Vector3(1.0, 1.0, 1.0);
    NOISE_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  };
  ~StereoHandler() {};


  void updateCurrentPoseID(int id)             {to_id_ = id;};
  void updatePreviousPoseID()                  {from_id_ = to_id_;};
  void updateWorldRotation(gtsam::Rot3 rot) {world_rotation_ = rot;};

  void addPoseFactor(const geometry_msgs::PoseStampedConstPtr &pose_msg, 
                     gtsam::Values& initial_estimate, 
                     gtsam::NonlinearFactorGraph& graph);

  void addCloudFactor(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, 
                      gtsam::NonlinearFactorGraph& graph);
};


void StereoHandler::addPoseFactor(const geometry_msgs::PoseStampedConstPtr &msg, 
                                  gtsam::Values& initial_estimate,
                                  gtsam::NonlinearFactorGraph& graph)
{
  gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(msg->pose.orientation.x,
                                      msg->pose.orientation.y,
                                      msg->pose.orientation.z,
                                      msg->pose.orientation.w);

  gtsam::Point3 translation = gtsam::Point3(msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z);

  gtsam::Pose3 pose = gtsam::Pose3(world_rotation_*rotation, 
                                   world_rotation_*translation);

  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(from_id_), 
                                               gtsam::symbol_shorthand::X(to_id_), 
                                               pose, 
                                               NOISE_)); 

  if (! initial_estimate.exists(gtsam::symbol_shorthand::X(to_id_)))
    initial_estimate.insert(gtsam::symbol_shorthand::X(to_id_), pose);
}


void StereoHandler::addCloudFactor(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, 
                                   gtsam::NonlinearFactorGraph& graph)
{
  NotImplementedError(__func__, __FILE__);
}

