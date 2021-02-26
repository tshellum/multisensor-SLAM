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
#include <gtsam/navigation/CombinedImuFactor.h>

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
  double association_threshold_; // in seconds
  std::map<ros::Time, int> stamped_pose_ids_;
  gtsam::Pose3 pose_;

public:
  Backend() 
  : pose_id_(0),
    association_threshold_(0.01),
    pose_(gtsam::Pose3::identity()),
    buffer_size_(1000),
    optimize_timer_(nh_.createTimer(ros::Duration(5), &Backend::callback, this)), 
    updated_(true),
    graph_filename_("graph.dot"), result_path_(ros::package::getPath("backend") + "/../../results/")
  {
    world_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/backend/pose_world", buffer_size_);
  };

  ~Backend() 
  {
    isam2_.saveGraph(result_path_ + graph_filename_);
  }

  int getPoseID() const                   { return pose_id_; }
  gtsam::Values& getValues()              { return new_values_;  }
  gtsam::NonlinearFactorGraph& getGraph() { return new_factors_; }

  int incrementPoseID() { return ++pose_id_; }
  void registerStampedPose(ros::Time stamp, int id) { stamped_pose_ids_[stamp] = id; }
  void isUpdated() { updated_ = true; }

  geometry_msgs::PoseStamped generateMsg();
  void callback(const ros::TimerEvent& event);

  template <typename Value>
  void insertValue(gtsam::Key pose_key, Value value);
  
  std::pair<int, bool> searchAssociatedPose(ros::Time pose_stamp, ros::Time prev_pose_stamp);
};


void Backend::callback(const ros::TimerEvent& event)
{
  if (new_values_.size())
  {
    new_values_.print();

    isam2_.update(new_factors_, new_values_);
    // for (int i = 0; i < optNum; ++i)
    //   isam2_.update(); 
      
    gtsam::Values current_estimate = isam2_.calculateBestEstimate();
    pose_ = current_estimate.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id_));

    // pose_.print();

    new_factors_.resize(0);
    new_values_.clear();

    world_pose_pub_.publish(generateMsg());
  }
}


geometry_msgs::PoseStamped Backend::generateMsg()
{
  tf2::Stamped<Eigen::Affine3d> tf2_stamped_T(
    Eigen::Isometry3d{pose_.matrix()}, 
    ros::Time::now(), 
    "backend_pose_world"
  );
  geometry_msgs::PoseStamped stamped_pose_msg = tf2::toMsg(tf2_stamped_T);

  return stamped_pose_msg;
}


template <typename Value>
void Backend::insertValue(gtsam::Key pose_key, Value value)
{
  if (! new_values_.exists(pose_key))
    new_values_.insert(pose_key, value); 
}


std::pair<int, bool> Backend::searchAssociatedPose(ros::Time pose_stamp, ros::Time prev_pose_stamp = ros::Time(0.0))
{    
  std::map<ros::Time, int>::iterator stamped_pose_id = stamped_pose_ids_.begin();
  std::map<ros::Time, int>::iterator prev = stamped_pose_ids_.begin();

  while (stamped_pose_id != stamped_pose_ids_.end())
  {
    if (pose_stamp < stamped_pose_id->first)
      break;

    prev = stamped_pose_id++;;
  }

  double prev_diff = std::abs((pose_stamp - prev->first).toSec());
  double diff = std::abs((pose_stamp - stamped_pose_id->first).toSec());

  if ( (prev_diff < association_threshold_) 
    || (diff < association_threshold_) ) 
  {
    if (diff < prev_diff)
      return std::make_pair(stamped_pose_id->second, true); // Next is associated
    
    if ( (diff > prev_diff) && (prev->first != prev_pose_stamp)) // Ensure that previous association isn't the actual previous pose 
      return std::make_pair(prev->second, true);            // Prev is associated
  }

  if (stamped_pose_id != stamped_pose_ids_.end()) 
    return std::make_pair(stamped_pose_id->second, false);  // Returns id of next pose
  else
  {
    stamped_pose_ids_[pose_stamp] = ++pose_id_; 
    return std::make_pair(pose_id_, false);;                // New pose
  }
}



} // namespace backend