/*** ROS packages ***/
#include <ros/ros.h>
#include <ros/package.h>

#include <tf2_eigen/tf2_eigen.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

/*** C++ packages ***/
#include <string>
#include <fstream>

/*** Local ***/
#include "motion2file/utils.h"


class Motion2file
{
  private:
    ros::NodeHandle nh_; 
    ros::Subscriber gnss_sub_;
    ros::Subscriber vo_sub_;

    std::string gt_path_;
    std::string slam_path_;

  public:
    Motion2file() 
    {
 			gnss_sub_ = nh_.subscribe("gt_topic", 1, &Motion2file::gt_callback, this);
 			vo_sub_   = nh_.subscribe("vo_topic", 1, &Motion2file::vo_callback, this);

      // Construct classes
      std::string result_path = ros::package::getPath("motion2file") + "/../../../results/";
      gt_path_   = result_path + "stamped_groundtruth.txt";
      slam_path_ = result_path + "stamped_traj_estimate.txt";

      // Clear files
      std::ofstream vo_results;
      vo_results.open(slam_path_, std::ofstream::out | std::ofstream::trunc);
      vo_results.close();

      std::ofstream gt_results;
      gt_results.open(gt_path_, std::ofstream::out | std::ofstream::trunc);
      gt_results.close();
    }

    ~Motion2file() {}



    void gt_callback(const tf2_msgs::TFMessage msg) 
    {
      ros::Time stamp = msg.transforms[0].header.stamp;

      Eigen::Vector3d t;
      tf2::fromMsg(msg.transforms[0].transform.translation, t);

      Eigen::Quaterniond q;
      tf2::fromMsg(msg.transforms[0].transform.rotation, q);

      appendToFile(gt_path_, stamp, t, q);
    }


    void vo_callback(const geometry_msgs::PoseStamped msg) 
    {
      ros::Time stamp = msg.header.stamp;

      Eigen::Vector3d t;
      tf2::fromMsg(msg.pose.position, t);

      Eigen::Quaterniond q;
      tf2::fromMsg(msg.pose.orientation, q);

      appendToFile(slam_path_, stamp, t, q);
    }

};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion2file");
	ROS_INFO("\n\n\n ----- Starting motion2file ----- \n");

	Motion2file object;

  ros::spin();
}