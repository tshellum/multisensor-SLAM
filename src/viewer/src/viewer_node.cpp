#include <ros/ros.h>
#include "viewer/VO_msg.h"

#include "viewer/viewer.h"


class Viewer
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Timer viewer_timer_;    

  Visualization viz;

public:
  Viewer()
  {
    sub_ = nh_.subscribe("topic", 1, &Viewer::callback, this); 
    viewer_timer_ = nh_.createTimer(ros::Duration(0.1), &Viewer::timerCB, this);
  }

  void callback(const viewer::VO_msg msg)
  {
    viz.readPose(msg);
    viz.readCloud(msg);

    viz.setEnvironment();
    viz.addCamera();
    
    viz.spinOnce();
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