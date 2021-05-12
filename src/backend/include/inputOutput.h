#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <gtsam/geometry/Pose3.h>

#include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>



geometry_msgs::PoseStamped generatePoseMsg(ros::Time stamp, gtsam::Pose3 pose)
{
  tf2::Stamped<Eigen::Affine3d> tf2_stamped_T(
    Eigen::Isometry3d{pose.matrix()}, 
    stamp, 
    "map"
  );
  geometry_msgs::PoseStamped stamped_pose_msg = tf2::toMsg(tf2_stamped_T);

  return stamped_pose_msg;
}


sensor_msgs::PointCloud2 generateCloudMsg(ros::Time stamp, gtsam::Values result)
{
  pcl::PointCloud<pcl::PointXYZ> global_cloud;

  int size = 0;
  for (auto value : result)
  {
    gtsam::Symbol symb(value.key);
    if (symb.chr() == 'l')
    {
      gtsam::Vector3 landmark = result.at<gtsam::Vector3>(value.key);
      global_cloud.points.push_back( pcl::PointXYZ(landmark.x(), landmark.y(), landmark.z()) );
      size++;
    }
  }

  global_cloud.width = size;
  global_cloud.height = 1;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(global_cloud, cloud_msg);

  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = "map";

  return cloud_msg;
}



geometry_msgs::TransformStamped generateOdomOriginMsg(ros::Time stamp, gtsam::Pose3 pose, std::string tf_name)
{
  geometry_msgs::TransformStamped msg = tf2::eigenToTransform(Eigen::Isometry3d{pose.matrix()});
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  msg.child_frame_id = tf_name;
  
  return msg;
}