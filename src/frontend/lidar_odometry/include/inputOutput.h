#pragma once

/*** ROS packages ***/
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>



/***** OUTPUT *****/
geometry_msgs::PoseStamped toPoseStamped(ros::Time stamp, Eigen::Affine3d T)
{
    tf2::Stamped<Eigen::Affine3d> tf2_stamped_T(T, stamp, "relative_pose");
    geometry_msgs::PoseStamped stamped_pose_msg = tf2::toMsg(tf2_stamped_T);

    return stamped_pose_msg;
}



