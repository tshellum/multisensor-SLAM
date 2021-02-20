/*** ROS packages ***/
#include <ros/ros.h> 
#include <ros/package.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/*** GTSAM packages ***/
#include <gtsam/nonlinear/ISAM2.h>

/*** Class packages ***/
#include "backend/gnss_handler.h"
#include "backend/stereo_handler.h"
#include "backend/imu_handler.h"

/*** Standard library ***/ 
#include <fstream>


class Backend
{
private:
  // Nodes
  int buffer_size_;
  ros::NodeHandle nh_;
  ros::Subscriber pulled_image_sub_;
  ros::Subscriber gnss_sub_;
  ros::Subscriber imu_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;


  // File
  std::string result_path_;
  std::string filename_;


  // ISAM2
  gtsam::ISAM2 isam2_; 
  gtsam::ISAM2Params isam2_params_; 

  // Graph 
  gtsam::Values new_values_; 
  gtsam::NonlinearFactorGraph new_factors_;
  
  int pose_id_;

  // Current state
  gtsam::Pose3 pose_;

  // Classes
  GNSSHandler   gnss_;
  IMUHandler    imu_;
  StereoHandler vo_;

public:
  Backend() 
  : pose_id_(0),
    buffer_size_(1),
    filename_("graph.dot"), result_path_(ros::package::getPath("stereo_frontend") + "/../../results/"),
    imu_(pose_id_, new_values_, new_factors_)
  {
    gnss_sub_         = nh_.subscribe("gnss_topic", 1, &Backend::gnss_callback, this);
    imu_sub_          = nh_.subscribe("imu_topic", 1, &Backend::imu_callback, this);
    pulled_image_sub_ = nh_.subscribe("pulled_image_flag_topic", 1, &Backend::preintegrated_imu_callback, this);
    pose_sub_.subscribe(nh_, "stereo_pose_relative_topic", 1);
    cloud_sub_.subscribe(nh_, "stereo_cloud_topic", 1);
    sync_.reset(new Sync(SyncPolicy(10), pose_sub_, cloud_sub_));
    sync_->registerCallback(boost::bind(&Backend::stereo_callback, this, _1, _2));

  };

  ~Backend() 
  {
    saveGraph();
  };


  void gnss_callback(const tf2_msgs::TFMessage& msg);
  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg);
  void preintegrated_imu_callback(const std_msgs::HeaderPtr& pulled_image_flag);
  void stereo_callback(const geometry_msgs::PoseStampedConstPtr& pose_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

  void addNewFactors2Graph();
  void optimize();

  void saveGraph();
};


void Backend::gnss_callback(const tf2_msgs::TFMessage& msg)
{
  pose_id_++;
  imu_.addPreintegrated2Graph(pose_id_, new_values_, new_factors_);
  gnss_.addPose2Graph(pose_id_, msg, new_values_, new_factors_);
}


void Backend::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
  imu_.preintegrateMeasurement(imu_msg);

}


void Backend::preintegrated_imu_callback(const std_msgs::HeaderPtr& pulled_image_flag)
{
  pose_id_++;
  imu_.addPreintegrated2Graph(pose_id_, new_values_, new_factors_);
  vo_.updateCurrentPoseID(pose_id_);
}


void Backend::stereo_callback(const geometry_msgs::PoseStampedConstPtr& pose_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  vo_.addPose2Graph(pose_msg, new_values_, new_factors_);
  // vo_.addCloud2Graph(cloud_msg, new_factors_);
  vo_.updatePreviousPoseID();

  addNewFactors2Graph();
  optimize();
  vo_.updateWorldRotation(pose_.rotation());
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


void Backend::saveGraph()
{
  isam2_.saveGraph(result_path_ + filename_);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "backend_node");
	ROS_INFO("\n\n\n ----- Starting backend ----- \n");

  Backend backend;

	ros::spin();
}

