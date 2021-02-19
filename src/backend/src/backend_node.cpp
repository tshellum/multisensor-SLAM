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
  int _buffer_size;
  ros::NodeHandle _nh;
  ros::Subscriber _pulled_image_sub;
  ros::Subscriber _gnss_sub;
  ros::Subscriber _imu_sub;
  message_filters::Subscriber<geometry_msgs::PoseStamped> _pose_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> _cloud_sub;

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> _sync;


  // File
  std::string _result_path;
  std::string _filename;


  // ISAM2
  gtsam::ISAM2 _isam2; 
  gtsam::ISAM2Params _isam2_params; 

  // Graph 
  gtsam::Values _initial_estimate; 
  // gtsam::Values _current_estimate;
  gtsam::NonlinearFactorGraph _graph;
  
  int _pose_id;
  // int _prev_odometry_id;
  // int _cur_odometry_id;

  std::vector<double> _timestamps;
  std::map<double, int> _timestamped_ids;

  // Classes
  GNSSHandler   _gnss;
  IMUHandler    _imu;
  StereoHandler _stereo;

public:
  Backend() 
  : _pose_id(0),
    _buffer_size(1),
    _filename("graph.dot"), _result_path(ros::package::getPath("stereo_frontend") + "/../../results/"),
    _imu(_initial_estimate, _graph)
  {
    _gnss_sub         = _nh.subscribe("gnss_topic", 1, &Backend::gnss_callback, this);
    _imu_sub          = _nh.subscribe("imu_topic", 1, &Backend::imu_callback, this);
    _pulled_image_sub = _nh.subscribe("pulled_image_flag_topic", 1, &Backend::preintegrated_imu_callback, this);
    _pose_sub.subscribe(_nh, "stereo_pose_relative_topic", 1);
    _cloud_sub.subscribe(_nh, "stereo_cloud_topic", 1);
    _sync.reset(new Sync(SyncPolicy(10), _pose_sub, _cloud_sub));
    _sync->registerCallback(boost::bind(&Backend::stereo_callback, this, _1, _2));

  };

  ~Backend() 
  {
    saveGraph();
  };


  void gnss_callback(const tf2_msgs::TFMessage& msg);
  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg);
  void preintegrated_imu_callback(const std_msgs::HeaderPtr& pulled_image_flag);
  void stereo_callback(const geometry_msgs::PoseStampedConstPtr& pose_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

  void optimize();

  void saveGraph();
};



void Backend::gnss_callback(const tf2_msgs::TFMessage& msg)
{
  _gnss.addPose2Graph(++_pose_id, msg, _graph);
  _imu.addPreintegrated2Graph(_pose_id, _initial_estimate, _graph);
}


void Backend::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
  _imu.preintegrateMeasurement(imu_msg);

}


void Backend::preintegrated_imu_callback(const std_msgs::HeaderPtr& pulled_image_flag)
{
  _imu.addPreintegrated2Graph(++_pose_id, _initial_estimate, _graph);
  _stereo.setCurrentPoseID(_pose_id);
}


void Backend::stereo_callback(const geometry_msgs::PoseStampedConstPtr& pose_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  _stereo.addPose2Graph(pose_msg, _graph);
  _stereo.addCloud2Graph(_pose_id, cloud_msg, _graph);
}


void Backend::optimize()
{
  // gtsam::ISAM2Result result = isam.update(graph, initialEstimate);
  // int numberOfUpdates = 50;
  // //Optimerer over resultatene 50 ganger for aa passe paa at vi finner bra resultat
  // for (int i = 0 ; i < numberOfUpdates; ++i){
  //   isam.update();
  // }

  // //Gir oss nytt estimate
  // Values currentEstimate = isam.calculateBestEstimate();

  // total_pose2 = currentEstimate.at<Pose3>(X(pose_counter));
}


void Backend::saveGraph()
{
  // graph.print(": "); 
  // graph.printErrors(result); 
  std::ofstream f(_result_path + _filename); 
  _graph.saveGraph(f);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "backend_node");
	ROS_INFO("\n\n\n ----- Starting backend ----- \n");

  Backend backend;

	ros::spin();
}

