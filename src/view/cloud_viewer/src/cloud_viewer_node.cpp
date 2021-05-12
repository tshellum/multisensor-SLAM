#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include "viewer/viewer.h"


class Viewer
{
private:
  ros::NodeHandle _nh;
  message_filters::Subscriber<geometry_msgs::PoseStamped> _pose_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> _cloud_sub;

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> _sync;

  ros::Timer _viewer_timer;    

  Visualization viz;

  bool initialized_;

public:
  Viewer()
  : initialized_(false)
  {
    _pose_sub.subscribe(_nh, "pose_topic", 1);
    _cloud_sub.subscribe(_nh, "point_cloud_topic", 1);
    _sync.reset(new Sync(SyncPolicy(10), _pose_sub, _cloud_sub));
    _sync->registerCallback(boost::bind(&Viewer::callback, this, _1, _2));

    _viewer_timer = _nh.createTimer(ros::Duration(0.1), &Viewer::timerCB, this);
  }

  void callback(const geometry_msgs::PoseStampedConstPtr &pose_msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    // viz.updatePoseRelative(*pose_msg);
    viz.updatePoseWorld(*pose_msg);
    viz.readCloud(*cloud_msg);
    viz.setEnvironment();
    if (! initialized_)
      viz.addOrigin();    
    else
      viz.addCamera();
    
    // viz.show();
    viz.spinOnce();

    initialized_ = true;
  }

  void timerCB(const ros::TimerEvent&)
  {
      if(viz.wasStopped())
          {
              ros::shutdown();
          }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "viewer");
	ROS_INFO("\n\n\n ----- Starting Viewer ----- \n");

  Viewer viewer;

  ros::spin();
}