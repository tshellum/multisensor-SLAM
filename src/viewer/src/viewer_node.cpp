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

public:
  Viewer()
  {
    _pose_sub.subscribe(_nh, "pose_relative_topic", 1);
    _cloud_sub.subscribe(_nh, "point_cloud_topic", 1);
    _sync.reset(new Sync(SyncPolicy(10), _pose_sub, _cloud_sub));
    _sync->registerCallback(boost::bind(&Viewer::callback, this, _1, _2));
  }

  void callback(const geometry_msgs::PoseStampedConstPtr &pose_msg, const sensor_msgs::PointCloud2ConstPtr &pc_msg)
  {
    ROS_INFO("Synchronization successful");
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "viewer");
	ROS_INFO("\n\n\n ----- Starting Viewer ----- \n");

  Viewer viewer;

  ros::spin();
}