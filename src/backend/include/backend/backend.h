#pragma once 

/*** ROS packages ***/
#include <ros/ros.h> 
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/*** GTSAM packages ***/
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

/*** Standard library ***/ 
#include <fstream>

namespace backend
{

class Backend
{
private:
  // Nodes
  ros::NodeHandle nh_;
  ros::Publisher  world_pose_pub_;
  ros::Timer optimize_timer_; 
  const int buffer_size_;

  // File
  std::string result_path_;
  std::string graph_filename_;

  // ISAM2
  gtsam::ISAM2 isam2_; 
  gtsam::ISAM2Params isam2_params_; 

  // Graph 
  bool updated_; 
  gtsam::Values new_values_; 
  gtsam::NonlinearFactorGraph new_factors_;
  
  // Current state
  int pose_id_;
  std::map<ros::Time, int> stamped_pose_ids_;
  gtsam::Pose3 pose_;

public:
  Backend(); 
  ~Backend(); 

  int getPoseID() const                   { return pose_id_; }
  gtsam::Values& getValues()              { return new_values_;  }
  gtsam::NonlinearFactorGraph& getGraph() { return new_factors_; }

  int incrementPoseID() { return ++pose_id_; }
  void registerStampedPose(ros::Time stamp, int id) { stamped_pose_ids_[stamp] = id; }
  void isUpdated() { updated_ = true; }

  geometry_msgs::PoseStamped generateMsg();
  void callback(const ros::TimerEvent& event);

  int searchAssociatedPose(ros::Time pose_stamp);
};


} // namespace backend