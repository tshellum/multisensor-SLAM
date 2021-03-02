#include "backend/backend.h"


namespace backend
{

Backend::Backend() 
  : pose_id_(0),
  pose_(gtsam::Pose3::identity()),
  buffer_size_(1000),
  optimize_timer_(nh_.createTimer(ros::Duration(5), &Backend::callback, this)), 
  updated_(true),
  graph_filename_("graph.dot"), result_path_(ros::package::getPath("backend") + "/../../results/")
{
  world_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/backend/pose_world", buffer_size_);
};

Backend::~Backend() 
{
  isam2_.saveGraph(result_path_ + graph_filename_);
}


void Backend::callback(const ros::TimerEvent& event)
{
  if (new_values_.size())
  {
    // new_values_.print();

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


int Backend::searchAssociatedPose(ros::Time pose_stamp)
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

  if ( (prev_diff < 0.01) || (diff < 0.01) ) // is associated
  {
    if (diff < prev_diff)
      return stamped_pose_id->second;
    else
      return prev->second;
  }

  if (stamped_pose_id != stamped_pose_ids_.end())
    return stamped_pose_id->second;
  else
  {
    stamped_pose_ids_[pose_stamp] = ++pose_id_;
    return pose_id_;
  }
}


} // namespace backend

