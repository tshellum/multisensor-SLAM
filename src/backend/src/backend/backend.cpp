#include "backend/backend.h"


namespace backend
{

Backend::Backend() 
  : pose_id_(0), time_prev_pose_relative_(ros::Time(0.0)),
    updated_(false), nav_status_(false), imu_status_(std::make_pair(false, false)),
    association_threshold_(0.01),
    pose_(gtsam::Pose3::identity()),
    graph_filename_("graph.dot"), result_path_(ros::package::getPath("backend") + "/../../results/"),
    buffer_size_(1000),
    optimize_timer_(nh_.createTimer(ros::Duration(0.1), &Backend::callback, this)),
    world_pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("/backend/pose_world", buffer_size_)) 
  {}

Backend::~Backend() 
{
  isam2_.saveGraph(result_path_ + graph_filename_);
}


void Backend::callback(const ros::TimerEvent& event)
{
  if (updated_ || (imu_status_.first && imu_status_.second))
  {
    ros::Time tic = ros::Time::now();
    
    // new_values_.print();

    isam2_.update(new_factors_, new_values_);
    // for (int i = 0; i < optNum; ++i)
    //   isam2_.update(); 
      
    gtsam::Values current_estimate = isam2_.calculateBestEstimate();
    pose_ = current_estimate.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id_));

    pose_.print();
    world_pose_pub_.publish(generateMsg());


    // Reset parameters
    new_factors_.resize(0);
    new_values_.clear();
    stamped_pose_ids_.clear(); // Alternatively [begin, previous relative pose]
    updated_ = false;
    imu_status_.second = false;

    ros::Time toc = ros::Time::now();
    ROS_INFO_STREAM("Time per iteration: " <<  (toc - tic) << "\n");
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

  // Check if associated relative pose. If associated, check if previous or next is closest
  if ( updated_
    && ((prev_diff < association_threshold_) 
    || (diff < association_threshold_)) ) 
  {
    if ( (diff > prev_diff) && (prev->first > prev_pose_stamp)) // Ensure that previous association isn't the actual previous pose 
    {
      stamped_pose_ids_[pose_stamp] = prev->second; 
      return std::make_pair(prev->second, true);            // Prev is associated
    }

    if (stamped_pose_id->first > prev_pose_stamp)
    {
      stamped_pose_ids_[pose_stamp] = stamped_pose_id->second; 
      return std::make_pair(stamped_pose_id->second, true); // Next is associated
    }
  }

  // Not associated pose --> Increase pose_id. 
  stamped_pose_ids_[pose_stamp] = ++pose_id_; 
  return std::make_pair(pose_id_, false);                   // New pose at end
}


} // namespace backend

