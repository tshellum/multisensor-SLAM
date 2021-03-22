#pragma once

/*** ROS packages ***/
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>

/*** PCL packages ***/
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


/***** INPUT *****/
cv::Mat readGray(const sensor_msgs::ImageConstPtr img_msg)
{
  //Frames
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr  = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

  return cv_ptr->image;
}


/***** OUTPUT *****/
geometry_msgs::PoseStamped toPoseStamped(ros::Time stamp, Eigen::Affine3d T)
{
    tf2::Stamped<Eigen::Affine3d> tf2_stamped_T(T, stamp, "relative_pose");
    geometry_msgs::PoseStamped stamped_pose_msg = tf2::toMsg(tf2_stamped_T);

    return stamped_pose_msg;
}



sensor_msgs::PointCloud2 toPointCloud2Msg(ros::Time stamp, pcl::PointCloud<pcl::PointXYZ> cloud)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_out;
  for(int n = 0; n < cloud.points.size(); n++)
	{
    pcl::PointXYZ pt;
    pt.x = cloud.points[n].z;
    pt.y = cloud.points[n].x;
    pt.z = cloud.points[n].y;

    cloud_out.points.push_back(pt);
	}

  cloud_out.width = (int)cloud_out.points.size();
  cloud_out.height = 1;
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_out, cloud_msg);

  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = "point_cloud";

  return cloud_msg;
}



/***** FORMATTING *****/
std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> convert(std::vector<cv::KeyPoint> image_points_left,
                                                                      std::vector<cv::KeyPoint> image_points_right)
{
  std::vector<cv::Point2f> pts_l, pts_r;
  cv::KeyPoint::convert(image_points_left, pts_l);
  cv::KeyPoint::convert(image_points_right, pts_r);

  return std::make_pair(pts_l, pts_r);
}

std::pair<std::vector<cv::Point3f>, std::vector<cv::Point2f>> find3D2DCorrespondences(pcl::PointCloud<pcl::PointXYZ> world_points,
                                                                                      std::vector<cv::KeyPoint> image_points)
{
  std::vector<cv::Point2f> image_point_correspondences; 
  std::vector<cv::Point3f> world_point_correspondences;
  
  if ( ! image_points.empty() && ! world_points.empty() )
  {
    for(int i = 0; i < image_points.size(); i++)
    {
      for(int j = 0; j < world_points.size(); j++)
      {
        if ( image_points[i].class_id == world_points.points[j].data[0] )
        {
          cv::Point3f wrld_pt;
          wrld_pt.x = world_points.points[j].x;
          wrld_pt.y = world_points.points[j].y;
          wrld_pt.z = world_points.points[j].z;

          image_point_correspondences.push_back(image_points[i].pt);
          world_point_correspondences.push_back(wrld_pt);
          break;
        }
      }
    }
  }

  return std::make_pair(world_point_correspondences, image_point_correspondences);
}


std::pair<std::vector<cv::Point3f>, std::vector<cv::Point3f>> find3D3DCorrespondences(pcl::PointCloud<pcl::PointXYZ> world_points_prev, 
                                                                                      pcl::PointCloud<pcl::PointXYZ> world_points_cur)
{
  std::vector<cv::Point3f> world_point_prev_corr;
  std::vector<cv::Point3f> world_point_cur_corr;
  
  if ( ! world_points_prev.empty() && ! world_points_cur.empty() )
  {
    for(int i = 0; i < world_points_prev.size(); i++)
    {
      for(int j = 0; j < world_points_cur.size(); j++)
      {
        if ( world_points_prev.points[j].data[0] == world_points_cur.points[j].data[0] )
        {
          cv::Point3f wrld_pt_prev;
          wrld_pt_prev.x = world_points_prev.points[j].x;
          wrld_pt_prev.y = world_points_prev.points[j].y;
          wrld_pt_prev.z = world_points_prev.points[j].z;

          cv::Point3f wrld_pt_cur;
          wrld_pt_cur.x = world_points_cur.points[j].x;
          wrld_pt_cur.y = world_points_cur.points[j].y;
          wrld_pt_cur.z = world_points_cur.points[j].z;

          world_point_prev_corr.push_back(wrld_pt_prev);
          world_point_cur_corr.push_back(wrld_pt_cur);
          break;
        }
      }
    }
  }

  return std::make_pair(world_point_prev_corr, world_point_cur_corr);
}