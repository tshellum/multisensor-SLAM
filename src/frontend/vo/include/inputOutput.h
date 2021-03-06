#pragma once

/*** ROS packages ***/
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include "vo/VO_msg.h"
#include "vo/PointID_msg.h"

#include "vo/VSLAM_msg.h"
#include "vo/IDPoint3D_msg.h"
#include "vo/IDPoint2D_msg.h"

/*** OpenCV packages ***/
#include <opencv2/core/eigen.hpp>

/*** PCL packages ***/
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>



/***** INPUT *****/
cv::Mat readGray(const sensor_msgs::ImageConstPtr img_msg)
{
  //Frames
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

  return cv_ptr->image;
}


cv::Mat read(const sensor_msgs::ImageConstPtr img_msg)
{
  //Frames
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
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


vo::VO_msg generateMsg(ros::Time stamp, 
                       Eigen::Affine3d T_clcr,
                       std::vector<cv::Point3f> world_points,
                       std::vector<int> world_point_indices)
{
  vo::VO_msg vo_msg;
  vo_msg.header.frame_id = "vo";
  vo_msg.header.stamp = stamp;
  vo_msg.pose = tf2::toMsg( T_clcr );

  vo_msg.cloud_size = world_points.size();
  for(int i = 0; i < world_points.size(); i++)
  {
    geometry_msgs::Point pt_msg	= tf2::toMsg( 
      Eigen::Vector3d(
        world_points[i].x,
        world_points[i].y,
        world_points[i].z
      )
    );
    
    vo::PointID_msg pt_id_msg;
    pt_id_msg.world_point = pt_msg;
    pt_id_msg.id = world_point_indices[i];
    vo_msg.cloud.push_back(pt_id_msg);
  }

  return vo_msg;
}



vo::VO_msg generateMsgInBody(ros::Time stamp, 
                             Eigen::Affine3d T_clcr,
                             std::vector<cv::Point3f> world_points,
                             std::vector<int> world_point_indices,
                             std::string frame = "")
{
  Eigen::Matrix4d T_bc = Eigen::Matrix4d::Identity();
  if (frame == "NED")
  {
    T_bc << 0, 0, 1, 0,
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
  }
  
  if (frame == "ENU")
  {  
    T_bc <<  0,  0, 1, 0,
            -1,  0, 0, 0,
             0, -1, 0, 0,
             0,  0, 0, 1;
  }
  
  Eigen::Matrix4d T_cb = T_bc.transpose();

  vo::VO_msg vo_msg;
  vo_msg.header.frame_id = "vo";
  vo_msg.header.stamp = stamp;
  vo_msg.pose = tf2::toMsg( Eigen::Affine3d{T_bc * T_clcr * T_cb} );

  vo_msg.cloud_size = world_points.size();
  for(int i = 0; i < world_points.size(); i++)
  {
    geometry_msgs::Point pt_msg	= tf2::toMsg( Eigen::Vector3d {
      T_bc.block<3,3>(0,0) * Eigen::Vector3d(
        world_points[i].x,
        world_points[i].y,
        world_points[i].z
      )
    } );

    vo::PointID_msg pt_id_msg;
    pt_id_msg.world_point = pt_msg;
    pt_id_msg.id = world_point_indices[i];
    vo_msg.cloud.push_back(pt_id_msg);
  }

  return vo_msg;
}


vo::IDPoint3D_msg toPt3DMsg(Eigen::Vector3d pt, int id)
{
  vo::IDPoint3D_msg msg;
  msg.x = pt.x();
  msg.y = pt.y();
  msg.z = pt.z();
  msg.id = id;

  return msg;
}


vo::IDPoint2D_msg toPt2DMsg(cv::Point2f pt, int id)
{
  vo::IDPoint2D_msg msg;
  msg.x = pt.x;
  msg.y = pt.y;
  msg.id = id;

  return msg;
}



vo::VSLAM_msg generateMsgInBody(ros::Time stamp,
                                  int sequence_id,
                                  Eigen::Affine3d T_r,
                                  bool is_keyframe,
                                  int keyframe_id,
                                  bool loop_found,
                                  int match_id,
                                  Eigen::Affine3d T_loop,
                                  std::vector<cv::KeyPoint> image_points_left,
                                  std::vector<cv::KeyPoint> image_points_right,
                                  std::vector<cv::Point3f> world_points,
                                  std::vector<int> world_point_indices,
                                  std::string frame = "")
{
  Eigen::Matrix4d T_bc = Eigen::Matrix4d::Identity();
  if (frame == "NED")
  {
    T_bc << 0, 0, 1, 0,
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
  }
  
  if (frame == "ENU")
  {  
    T_bc <<  0,  0, 1, 0,
            -1,  0, 0, 0,
             0, -1, 0, 0,
             0,  0, 0, 1;
  }
  
  Eigen::Matrix4d T_cb = T_bc.transpose();

  vo::VSLAM_msg vo_msg;
  vo_msg.header.frame_id = "vo";
  vo_msg.header.stamp = stamp;
  vo_msg.header.seq = sequence_id;
  vo_msg.pose = tf2::toMsg( Eigen::Affine3d{T_bc * T_r * T_cb} );

  vo_msg.is_keyframe.data = is_keyframe;
  vo_msg.keyframe_id = keyframe_id;

  vo_msg.loop_found.data = loop_found;
  vo_msg.match_id = match_id;
  vo_msg.pose_loop = tf2::toMsg( Eigen::Affine3d{T_bc * T_loop * T_cb} );


  vo_msg.landmark_size = world_points.size();
  for(int i = 0; i < world_points.size(); i++)
  {
    Eigen::Vector3d landmark_body = T_bc.block<3,3>(0,0) * Eigen::Vector3d(
      world_points[i].x,
      world_points[i].y,
      world_points[i].z
    );

    vo_msg.landmarks.push_back( toPt3DMsg(landmark_body, world_point_indices[i]) );
    vo_msg.lfeatures.push_back( toPt2DMsg(image_points_left[i].pt, image_points_left[i].class_id) );
    vo_msg.rfeatures.push_back( toPt2DMsg(image_points_right[i].pt, image_points_right[i].class_id) );
  }

  return vo_msg;
}




/***** FORMATTING *****/
Eigen::Matrix3d euler2rotationMatrix(double pitch, double yaw, double roll)
{
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ());

  return rotation;
}


std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> convert(std::vector<cv::KeyPoint> image_points_left,
                                                                      std::vector<cv::KeyPoint> image_points_right)
{
  std::vector<cv::Point2f> pts_l, pts_r;
  cv::KeyPoint::convert(image_points_left, pts_l);
  cv::KeyPoint::convert(image_points_right, pts_r);

  return std::make_pair(pts_l, pts_r);
}

std::pair<std::vector<cv::Point3f>, std::vector<cv::Point2f>> find3D2DCorrespondences(std::map<int, cv::Point3f> world_points,
                                                                                      std::vector<cv::KeyPoint> image_points)
{
  std::vector<cv::Point2f> image_point_correspondences; 
  std::vector<cv::Point3f> world_point_correspondences;
  
  if ( ! image_points.empty() && ! world_points.empty() )
  {
    for(int i = 0; i < image_points.size(); i++)
    {
      int id = image_points[i].class_id;

      if ( world_points.find(id) == world_points.end() ) 
        continue;

      cv::Point3f wrld_pt;
      wrld_pt.x = world_points[id].x;
      wrld_pt.y = world_points[id].y;
      wrld_pt.z = world_points[id].z;

      image_point_correspondences.push_back(image_points[i].pt);
      world_point_correspondences.push_back(wrld_pt);
    }
  }

  return std::make_pair(world_point_correspondences, image_point_correspondences);
}


std::pair<std::vector<cv::Point3f>, std::vector<cv::Point3f>> find3D3DCorrespondences(std::map<int, cv::Point3f> world_points_prev, 
                                                                                      std::map<int, cv::Point3f> world_points_cur)
{
  std::vector<cv::Point3f> world_point_prev_corr;
  std::vector<cv::Point3f> world_point_cur_corr;

  std::map<int, cv::Point3f>::iterator prev;
  for (prev = world_points_prev.begin(); prev != world_points_prev.end(); prev++)
  {
    int id = prev->first;
    if ( world_points_cur.find(id) == world_points_cur.end() )
      continue;

    world_point_prev_corr.push_back(world_points_prev[id]);
    world_point_cur_corr.push_back(world_points_prev[id]);

  }

  return std::make_pair(world_point_prev_corr, world_point_cur_corr);
}



std::pair<std::vector<cv::Point3f>, std::vector<cv::Point2f>> find3D2DCorrespondences(std::vector<cv::Point3f> world_points,
                                                                                      std::vector<int> world_point_indices,
                                                                                      std::vector<cv::KeyPoint> image_points)
{
  std::vector<cv::Point2f> image_point_correspondences; 
  std::vector<cv::Point3f> world_point_correspondences;
  
  if ( ! image_points.empty() && ! world_points.empty() )
  {
    for(int i = 0; i < image_points.size(); i++)
    {
      for(int j = 0; j < world_point_indices.size(); j++)
      {
        if ( image_points[i].class_id != world_point_indices[j] )
          continue;

        // Same ID
        image_point_correspondences.push_back(image_points[i].pt);
        world_point_correspondences.push_back(world_points[i]);
        break;
      }
    }
  }

  return std::make_pair(world_point_correspondences, image_point_correspondences);
}



void find3D2DCorrespondences(std::vector<cv::Point3f> world_points,
                             std::vector<int> world_point_indices,
                             std::vector<cv::KeyPoint> image_points_l,
                             std::vector<cv::KeyPoint> image_points_r,
                             std::vector<cv::Point3f>& world_point_corr,
                             std::vector<cv::Point2f>& image_points_l_corr,
                             std::vector<cv::Point2f>& image_points_r_corr)
{ 
  world_point_corr.clear();
  image_points_l_corr.clear();
  image_points_r_corr.clear();

  for(int i = 0; i < image_points_l.size(); i++)
  {
    for(int j = 0; j < world_point_indices.size(); j++)
    {
      if ( ( image_points_l[i].class_id != world_point_indices[j] ) 
        || ( image_points_r[i].class_id != world_point_indices[j] ))
        continue;

      // Same ID
      image_points_l_corr.push_back(image_points_l[i].pt);
      image_points_r_corr.push_back(image_points_r[i].pt);
      world_point_corr.push_back(world_points[j]);
      break;
    }
  }

}


std::pair<std::vector<cv::Point3f>, std::vector<cv::Point3f>> find3D3DCorrespondences(std::vector<cv::Point3f> world_points_prev,
                                                                                      std::vector<int> world_point_prev_indices,
                                                                                      std::vector<cv::Point3f> world_points_cur,
                                                                                      std::vector<int> world_point_cur_indices)
{
  std::vector<cv::Point3f> world_point_prev_corr;
  std::vector<cv::Point3f> world_point_cur_corr;

  if ( (! world_points_prev.empty()) && (! world_points_cur.empty()) )
  {
    for(int i = 0; i < world_points_prev.size(); i++)
    {
      for(int j = 0; j < world_points_cur.size(); j++)
      {
        if ( world_point_prev_indices[i] != world_point_cur_indices[j] )
          continue;

        world_point_prev_corr.push_back(world_points_prev[i]);
        world_point_cur_corr.push_back(world_points_cur[i]);
        break;        
      }
    }
  }
  
  return std::make_pair(world_point_prev_corr, world_point_cur_corr);
}



std::pair<std::vector<int>, std::vector<int>> find3D3DCorrespondenceIndices(std::vector<int> world_point_prev_indices,
                                                                            std::vector<int> world_point_cur_indices)
{
  std::vector<int> idx_prev;
  std::vector<int> idx_cur;

  if ( (! world_point_prev_indices.empty()) && (! world_point_cur_indices.empty()) )
  {
    for(int i = 0; i < world_point_prev_indices.size(); i++)
    {
      for(int j = 0; j < world_point_cur_indices.size(); j++)
      {
        if ( world_point_prev_indices[i] != world_point_cur_indices[j] )
          continue;

        idx_prev.push_back(world_point_prev_indices[i]);
        idx_cur.push_back(world_point_cur_indices[j]);
        break;        
      }
    }
  }
  
  return std::make_pair(idx_prev, idx_cur);
}




std::vector<cv::Point2f> findIndexMatchFeatures(std::vector<int> indices,
                                                std::vector<cv::KeyPoint> features)
{
  std::vector<cv::Point2f> feature_correspondences;

  if ( (! indices.empty()) && (! features.empty()) )
  {
    for(int i = 0; i < indices.size(); i++)
    {
      for(int j = 0; j < features.size(); j++)
      {
        if ( indices[i] != features[j].class_id )
          continue;

        feature_correspondences.push_back(features[i].pt);
        break;        
      }
    }
  }
  
  return feature_correspondences;
}



std::vector<cv::Point3f> findIndexMatchLandmarks(std::vector<int> indices,
                                                 std::vector<cv::Point3f> world_points,
                                                 std::vector<int> world_point_indices)
{
  std::vector<cv::Point3f> landmark_correspondences;

  if ( (! indices.empty()) && (! world_point_indices.empty()) )
  {
    for(int i = 0; i < indices.size(); i++)
    {
      for(int j = 0; j < world_point_indices.size(); j++)
      {
        if ( indices[i] != world_point_indices[j] )
          continue;

        landmark_correspondences.push_back(world_points[j]);
        break;        
      }
    }
  }
  
  return landmark_correspondences;
}