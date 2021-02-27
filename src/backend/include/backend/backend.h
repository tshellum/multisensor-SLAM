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
#include <gtsam/slam/BetweenFactor.h>

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

  // Sensor relevant variables
    int pose_id_prev_relative_;


public:
  Backend() 
  : pose_id_(0), pose_id_prev_relative_(0),
    association_threshold_(0.01),
    pose_(gtsam::Pose3::identity()),
    updated_(true),
    graph_filename_("graph.dot"), result_path_(ros::package::getPath("backend") + "/../../results/"),
    buffer_size_(1000),
    optimize_timer_(nh_.createTimer(ros::Duration(5), &Backend::callback, this)),
    world_pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("/backend/pose_world", buffer_size_)) 
  {};

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
  bool tryInsertValue(gtsam::Key pose_key, Value value, bool is_associated = false);
  
  std::pair<int, bool> searchAssociatedPose(ros::Time pose_stamp, ros::Time prev_pose_stamp = ros::Time(0.0));
  void insertBetweenFactor(int from_id, int to_id, bool is_associated, gtsam::Pose3 pose_relative, gtsam::noiseModel::Diagonal::shared_ptr noise);
};


void Backend::callback(const ros::TimerEvent& event)
{
  if (new_values_.size())
  {
    // Preintegrate all measurements
    // for (stamped_id = stamped_pose_ids_.begin(); stamped_id != stamped_pose_ids_.end(); stamped_id++)
    // {
      
    // }

    // new_values_.print();

    isam2_.update(new_factors_, new_values_);
    // for (int i = 0; i < optNum; ++i)
    //   isam2_.update(); 
      
    gtsam::Values current_estimate = isam2_.calculateBestEstimate();
    pose_ = current_estimate.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id_));

    pose_.print();

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


// template <typename Value>
// bool Backend::tryInsertValue(gtsam::Key pose_key, Value value, bool is_assosiated = false)
// {
//   if (! new_values_.exists(pose_key))
//   {
//     new_values_.insert(pose_key, value); 
//     return true;
//   }
//   else
//     return false;
// }



template <typename Value>
bool Backend::tryInsertValue(gtsam::Key pose_key, Value value, bool is_associated)
{
  if (! new_values_.exists(pose_key)) // Value doesn't exist, thus is inserted
  {
    new_values_.insert(pose_key, value); 
    return true;
  }

  if (is_associated)                  // Value exist and is associated, thus not inserted
    return false;
  else                                // Value exist and is not associated. This is a more complicated scenario. Insert at end to be swapped. Should only happend for poses                             
  {
    new_values_.insert(gtsam::symbol_shorthand::X(++pose_id_), value); 
    return true;
  }  
}


std::pair<int, bool> Backend::searchAssociatedPose(ros::Time pose_stamp, ros::Time prev_pose_stamp)
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


void Backend::insertBetweenFactor(int from_id, int to_id, bool is_associated, gtsam::Pose3 pose_relative, gtsam::noiseModel::Diagonal::shared_ptr noise)
{
  pose_id_prev_relative_ = to_id;

  gtsam::Key pose_key_from = gtsam::symbol_shorthand::X(from_id); 
  gtsam::Key pose_key_to   = gtsam::symbol_shorthand::X(to_id); 

  if ( (! isam2_.valueExists(gtsam::symbol_shorthand::X(0))) 
    && (! new_values_.exists(gtsam::symbol_shorthand::X(0))) ) // Insert a prior for the entire graph 
  {
    gtsam::Pose3 initial = gtsam::Pose3::identity(); // TODO: GNSS should initialize this
    new_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(0), initial, noise));
    tryInsertValue(gtsam::symbol_shorthand::X(0), initial);
  }


  if (is_associated || (to_id == pose_id_)) // Either connect to existing poses, or new pose
    new_factors_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(
        pose_key_from, pose_key_to, pose_relative, noise
      )
    );
  else
  {
    std::map<gtsam::Key, gtsam::Key> rekey_mapping;
    
    for (gtsam::Values::reverse_iterator value = new_values_.rbegin(); value != new_values_.rend();) 
    {
      // gtsam::Key prev_key = value->key;

      // value++;

      // value->value.print();
      // ROS_INFO_STREAM("value->key" << value->key);

      // rekey_mapping[prev_key] = value->key;
    }


    // new_factors_ = new_factors_.rekey(rekey_mapping); 
  }


  // Values - update(): https://gtsam.org/doxygen/4.0.0/a03871.html#a47bf2a64ee131889b02049b242640226
  // Graph - rekey(): http://www.borg.cc.gatech.edu/sites/edu.borg/html/a00181.html


}




} // namespace backend