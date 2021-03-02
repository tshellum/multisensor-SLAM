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
  ros::Time time_prev_pose_relative_;
  double association_threshold_; // in seconds
  std::map<ros::Time, int> stamped_pose_ids_;
  gtsam::Pose3 pose_;

  // Measurement states
  bool nav_status_;                  // is online
  std::pair<bool, bool> imu_status_; // <is online, is updated>

public:
  Backend(); 
  ~Backend(); 

  int getPoseID() const                   { return pose_id_; }
  gtsam::Values& getValues()              { return new_values_;  }
  gtsam::NonlinearFactorGraph& getGraph() { return new_factors_; }
  gtsam::ISAM2& getiSAM2()                { return isam2_; }
  bool checkNavStatus()                   { return nav_status_; }
  std::pair<bool, bool> checkIMUStatus()  { return imu_status_; }

  int  incrementPoseID() { return ++pose_id_; }
  void registerStampedPose(ros::Time stamp, int id) { stamped_pose_ids_[stamp] = id; }
  void isUpdated() { updated_ = true; }
  void updatedPreviousRelativeTimeStamp(ros::Time time) { time_prev_pose_relative_ = time; }
  void updateNavStatus(bool status) { nav_status_ = status; }
  void updateIMUOnlineStatus(bool status) { imu_status_.first = status; }
  void updatePreintegrationStatus(bool status) { imu_status_.second = status; }

  geometry_msgs::PoseStamped generateMsg();
  void callback(const ros::TimerEvent& event);

  template <typename Value>
  bool tryInsertValue(gtsam::Key pose_key, Value value);

  template <typename Value>
  void forceInsertValue(gtsam::Key pose_key, Value value);

  std::pair<int, bool> searchAssociatedPose(ros::Time pose_stamp, ros::Time prev_pose_stamp = ros::Time(0.0));

  template <typename FactorType>
  void addFactor(FactorType factor) { new_factors_.add(factor); }

};

template <typename Value>
bool Backend::tryInsertValue(gtsam::Key pose_key, Value value)
{
  if (! new_values_.exists(pose_key)) // Value doesn't exist, thus is inserted
  {
    new_values_.insert(pose_key, value); 
    return true;
  }
  else
    return false;
}


template <typename Value>
void Backend::forceInsertValue(gtsam::Key pose_key, Value value)
{
  new_values_.print();
  if (new_values_.exists(pose_key)) // Value exist, thus is deleted
  {
    new_values_.erase(pose_key);
    new_values_.insert(pose_key, value); 
  }
  else
    new_values_.insert(pose_key, value); 
}

} // namespace backend