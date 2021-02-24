#pragma once 

/*** ROS packages ***/
#include <ros/ros.h> 
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/*** GTSAM packages ***/
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

/*** Class packages ***/
// #include "backend/gnss_handler.h"
// #include "backend/stereo_handler.h"
// #include "backend/imu_handler.h"

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
  const int buffer_size_;
  ros::Rate loop_rate_;
  
  // File
  std::string result_path_;
  std::string graph_filename_;

  // ISAM2
  gtsam::ISAM2 isam2_; 
  gtsam::ISAM2Params isam2_params_; 

  // Graph 
  gtsam::Values new_values_; 
  gtsam::NonlinearFactorGraph new_factors_;
  
  int pose_id_;

  // Current state
  gtsam::Pose3 pose_;

public:
  Backend() 
  : pose_id_(0),
    buffer_size_(1000),
    loop_rate_(1),
    graph_filename_("graph.dot"), result_path_(ros::package::getPath("stereo_frontend") + "/../../results/")
  {
    world_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/backend/pose_world", buffer_size_);
  };

  ~Backend() 
  {
    isam2_.saveGraph(result_path_ + graph_filename_);
  }

  gtsam::Values& getValues() { return new_values_;  }
  gtsam::NonlinearFactorGraph& getGraph() { return new_factors_; }
  int getPoseID() const { return pose_id_; }

  geometry_msgs::PoseStamped generateMsg();
  void callback();

  void addNewFactors2Graph();
  void optimize();
};


geometry_msgs::PoseStamped Backend::generateMsg()
{
  geometry_msgs::PoseStamped pose_msg;
  return pose_msg;
}

void Backend::callback()
{
  // addNewFactors2Graph();
  // optimize();

  world_pose_pub_.publish(generateMsg());

  loop_rate_.sleep();
}




void Backend::addNewFactors2Graph()
{  
  // new_values_.print();

  isam2_.update(new_factors_, new_values_);

  new_factors_.resize(0);
  new_values_.clear();
}


void Backend::optimize()
{  
  gtsam::Values current_estimate = isam2_.calculateBestEstimate();
  pose_ = current_estimate.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id_));

  pose_.print();
}


} // namespace backend