/*** ROS packages ***/
#include <ros/ros.h> 
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

/*** GTSAM packages ***/
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>

/*** Class packages ***/
#include "backend/gnss_handler.h"
#include "backend/stereo_handler.h"


class Backend
{
private:
  // Nodes
  int _buffer_size;
  ros::NodeHandle _nh;
  ros::Subscriber _gnss_sub;
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
  gtsam::NonlinearFactorGraph _graph;

  // Classes
  int _pose_id;
  GNSSHandler   _gnss;
  StereoHandler _stereo;

public:
  Backend() 
  : _pose_id(0),
    _buffer_size(1),
    _filename("graph.dot"), _result_path(ros::package::getPath("stereo_frontend") + "/../../results/")
  {
    _gnss_sub  = _nh.subscribe("gnss_topic", 1, &Backend::gnss_callback, this);
    
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
  void stereo_callback(const geometry_msgs::PoseStampedConstPtr &pose_msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

  void optimize();

  void saveGraph();
};



void Backend::gnss_callback(const tf2_msgs::TFMessage& msg)
{
  _gnss.addPose2Graph(++_pose_id, msg, _graph);
}


void Backend::stereo_callback(const geometry_msgs::PoseStampedConstPtr &pose_msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  _stereo.addPose2Graph(++_pose_id, pose_msg, _graph);
  _stereo.addCloud2Graph(_pose_id, cloud_msg, _graph);
}


// void Backend::optimize()
// {
//   gtsam::ISAM2Result result = isam2_.update(graph_, initialEstimate_); 
// }


void Backend::saveGraph()
{
  // graph.print(": "); 
  // graph.printErrors(result); 

  _graph.saveGraph(_result_path + _filename);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "backend_node");
	ROS_INFO("\n\n\n ----- Starting backend ----- \n");

  Backend backend;

	ros::spin();
}

