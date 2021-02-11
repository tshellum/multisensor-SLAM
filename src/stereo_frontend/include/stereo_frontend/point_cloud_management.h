#pragma once


/*** OpenGV packages ***/
#include "opengv/include/triangulation/methods.hpp"
#include <opengv/include/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/include/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/include/triangulation/methods.hpp>


/*** ROS packages ***/
#include <sensor_msgs/PointCloud.h> 
#include <sensor_msgs/PointCloud2.h>


/*** PCL packages ***/
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


struct PointCloudFrame 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<cv::Point3f> cloud_cv;
    std::vector<int> ids;
    Eigen::Matrix3d R_wb;
    Eigen::Vector3d t_wb;
    int id_point_cloud;

    PointCloudFrame() : cloud(new pcl::PointCloud<pcl::PointXYZ>) {};
};



class PointCloudManager
{
private:
    PointCloudFrame _pc;
public:
    PointCloudManager(){};
    ~PointCloudManager(){};

		std::vector<cv::Point3f> getWorldPoints() {return _pc.cloud_cv;};
		pcl::PointCloud<pcl::PointXYZ> getPointCloud() {return *_pc.cloud;};

    void filterCorrespondences(std::vector<cv::KeyPoint> kpts_prev, std::vector<cv::KeyPoint> kpts_cur,
                               std::vector<cv::Point2f>& img_pts_prev, std::vector<cv::Point2f>& img_pts_cur, 
                               std::vector<cv::Point3f>& world_pts_prev);

    void setPointCloudHeader(std_msgs::Header header);
    void setExtrinsics(Eigen::Matrix3d R_wb, Eigen::Vector3d t_wb);

    void keypoint2bearing(std::vector<cv::KeyPoint> match_left, 
													std::vector<cv::KeyPoint> match_right,
													Eigen::Matrix3d K_l,
													Eigen::Matrix3d K_r,
													opengv::bearingVectors_t& bearing_l, 
													opengv::bearingVectors_t& bearing_r);

    // https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/CameraModels/KannalaBrandt8.cpp
    cv::Mat DLT(cv::Point2f pt_l, 
                    cv::Point2f pt_r,
                    cv::Mat P_l,
                    cv::Mat P_r);

    void triangulate(std::vector<cv::KeyPoint> match_left, 
                     std::vector<cv::KeyPoint> match_right,
                     cv::Mat P_cl,
                     cv::Mat P_cr,
                     Eigen::Affine3d T_wb_left);

    void triangulate(std::vector<cv::KeyPoint> match_left, 
                     std::vector<cv::KeyPoint> match_right,
                     cv::Mat P_cl,
                     cv::Mat P_cr,
                     cv::Mat R_wb,
                     cv::Mat t_wb);

		
    void mlpnp(std::vector<cv::KeyPoint> features,
							 Eigen::Affine3d T_b1b2,
               Eigen::Matrix3d K_l);

    sensor_msgs::PointCloud2 toPointCloud2Msg(std_msgs::Header header);
};



void PointCloudManager::filterCorrespondences(std::vector<cv::KeyPoint> kpts_prev, std::vector<cv::KeyPoint> kpts_cur,
                                              std::vector<cv::Point2f>& img_pts_prev, std::vector<cv::Point2f>& img_pts_cur, 
                                              std::vector<cv::Point3f>& world_pts_prev)
{
  // for(int i = 0; i < kpts_cur.size(); i++)
  // {
  //   ROS_INFO_STREAM("kpts_prev[i].class_id: \n" << kpts_prev[i].class_id);
  //   ROS_INFO_STREAM("kpts_cur[i].class_id: \n" << kpts_cur[i].class_id);
  // }

  if (_pc.ids.size())
  {
    // ROS_INFO_STREAM("filterCorrespondences - _pc.ids.size(): " << _pc.ids.size());
    // ROS_INFO_STREAM("img_pts_prev.size(): " << img_pts_prev.size());

    for(int i = 0; i < kpts_cur.size(); i++)
    {
      for(int j = 0; j < _pc.ids.size(); j++)
      {
        if(kpts_cur[i].class_id == _pc.ids[j])
        {
          img_pts_prev.push_back(kpts_prev[i].pt);
          img_pts_cur.push_back(kpts_cur[i].pt);
          world_pts_prev.push_back(_pc.cloud_cv[j]);
          
          break;
        }
      }
    }
  }
  else
  {
    // ROS_INFO("filterCorrespondences - else");

    cv::KeyPoint::convert(kpts_prev, img_pts_prev);
    cv::KeyPoint::convert(kpts_cur, img_pts_cur);
  }

  // ROS_INFO_STREAM("img_pts_prev.size(): " << img_pts_prev.size());
  // ROS_INFO_STREAM("img_pts_cur.size(): " << img_pts_cur.size());
  // ROS_INFO_STREAM("world_pts_prev.size(): " << world_pts_prev.size());

}


void PointCloudManager::setPointCloudHeader(std_msgs::Header header)
{
  _pc.cloud->header.frame_id = "stereo_point_cloud";
  _pc.cloud->header.stamp = _pc.cloud->header.stamp;
}



void PointCloudManager::setExtrinsics(Eigen::Matrix3d R_wb, Eigen::Vector3d t_wb)
{
    _pc.R_wb = R_wb;
    _pc.t_wb = t_wb;
}



cv::Mat PointCloudManager::DLT(cv::Point2f pt_l, 
                               cv::Point2f pt_r,
                               cv::Mat P_l,
                               cv::Mat P_r)
{
  /*********************************************************************
  Compute the 3D position of a single point from 2D correspondences.

    Args:
        uv1:    2D projection of point in image 1.
        uv2:    2D projection of point in image 2.
        P1:     Projection matrix with shape 3 x 4 for image 1.
        P2:     Projection matrix with shape 3 x 4 for image 2.

    Returns:
        X:      3D coordinates of point in the camera frame of image 1.
                (not homogeneous!)

    See HZ Ch. 12.2: Linear triangulation methods (p312)
  *********************************************************************/

  cv::Mat A(4,4,CV_64F);
  A.row(0) = pt_l.x*P_l.row(2)-P_l.row(0);
  A.row(1) = pt_l.y*P_l.row(2)-P_l.row(1);
  A.row(2) = pt_r.x*P_r.row(2)-P_r.row(0);
  A.row(3) = pt_r.y*P_r.row(2)-P_r.row(1);

  cv::Mat u,w,vt;
  cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
  cv::Mat pt3D = vt.row(3).t();  
  return pt3D.rowRange(0,3)/pt3D.at<double>(3); // / Homogenous to cartesian
}


void PointCloudManager::triangulate(std::vector<cv::KeyPoint> match_left, 
                                    std::vector<cv::KeyPoint> match_right,
                                    cv::Mat P_cl,
                                    cv::Mat P_cr,
                                    Eigen::Affine3d T_wb_left)
{
  _pc.cloud->clear();
  _pc.cloud_cv.clear();

  std::vector<cv::Point2f> point2D_left, point2D_right;
  cv::KeyPoint::convert(match_left, point2D_left);
  cv::KeyPoint::convert(match_right, point2D_right);

  cv::Mat t_wb, R_wb;
  Eigen::Vector3d t_wb_l = T_wb_left.translation();
  Eigen::Matrix3d R_wb_l = T_wb_left.linear();
  cv::eigen2cv(t_wb_l, t_wb);
  cv::eigen2cv(R_wb_l, R_wb);


  cv::Mat point3D_homogenous(4, point2D_left.size(), CV_64FC1);                                                 // https://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints
  cv::triangulatePoints(P_cl, P_cr, point2D_left, point2D_right, point3D_homogenous); // https://gist.github.com/cashiwamochi/8ac3f8bab9bf00e247a01f63075fedeb

  for(int i = 0; i < point2D_left.size(); i++)
  {
    // cv::convertPointsFromHomogeneous(point3D_homogenous.col(i).t(), pt3D);
    // cv::Mat pt3D = DLT(point2D_left[i], point2D_right[i], P_l, P_r);

    cv::Mat pt3Dh = point3D_homogenous.col(i); 
    cv::Mat pt3Dc = pt3Dh.rowRange(0,3)/pt3Dh.at<float>(3); // / Homogenous to cartesian
    cv::Mat pt3D; // / Homogenous to cartesian
    pt3Dc.convertTo(pt3D, CV_64FC1);

    if (pt3D.at<double>(2) > 0)
    {
      pt3D = t_wb + R_wb*pt3D;    

      pcl::PointXYZ pt;
      pt.x = pt3D.at<double>(0);
      pt.y = pt3D.at<double>(1);
      pt.z = pt3D.at<double>(2);

      cv::Point3f pt_cv;
      pt_cv.x = pt3D.at<double>(0);
      pt_cv.y = pt3D.at<double>(1);
      pt_cv.z = pt3D.at<double>(2);

      _pc.cloud->points.push_back(pt);
      _pc.cloud_cv.push_back(pt_cv);
      _pc.ids.push_back(match_left[i].class_id);
    }
    else
    {
      // ROS_INFO("triangulate() - Error");
    }
  }
}


void PointCloudManager::triangulate(std::vector<cv::KeyPoint> match_left, 
                                    std::vector<cv::KeyPoint> match_right,
                                    cv::Mat P_cl,
                                    cv::Mat P_cr,
                                    cv::Mat R_wb,
                                    cv::Mat t_wb)
{
  _pc.cloud->clear();
  _pc.cloud_cv.clear();
  _pc.ids.clear();

  std::vector<cv::Point2f> point2D_left, point2D_right;
  cv::KeyPoint::convert(match_left, point2D_left);
  cv::KeyPoint::convert(match_right, point2D_right);

  cv::Mat point3D_homogenous(4, point2D_left.size(), CV_64FC1);                                                 // https://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints
  cv::triangulatePoints(P_cl, P_cr, point2D_left, point2D_right, point3D_homogenous); // https://gist.github.com/cashiwamochi/8ac3f8bab9bf00e247a01f63075fedeb

  int n_err = 0;
  for(int i = 0; i < point2D_left.size(); i++)
  {
    // cv::convertPointsFromHomogeneous(point3D_homogenous.col(i).t(), pt3D);
    // cv::Mat pt3D = DLT(point2D_left[i], point2D_right[i], P_l, P_r);

    cv::Mat pt3Dh = point3D_homogenous.col(i); 
    cv::Mat pt3Dc = pt3Dh.rowRange(0,3)/pt3Dh.at<float>(3); // / Homogenous to cartesian
    cv::Mat pt3D; 
    pt3Dc.convertTo(pt3D, CV_64FC1);

    if (pt3D.at<double>(2) > 0)
    {
      cv::Point3f pt_cv;
      pt_cv.x = pt3D.at<double>(0);
      pt_cv.y = pt3D.at<double>(1);
      pt_cv.z = pt3D.at<double>(2);

      pt3D = t_wb + R_wb*pt3D;    

      pcl::PointXYZ pt;
      pt.x = pt3D.at<double>(0);
      pt.y = pt3D.at<double>(1);
      pt.z = pt3D.at<double>(2);

      _pc.cloud->points.push_back(pt);
      _pc.cloud_cv.push_back(pt_cv);
      _pc.ids.push_back(match_left[i].class_id);
    }
    else
    {
      n_err++;
      // ROS_INFO("triangulate() - Error");
    }
  }
  ROS_INFO_STREAM("triangulation() - Correct: " << _pc.cloud->points.size() << ", Erronous: " << n_err);
}




void PointCloudManager::keypoint2bearing(std::vector<cv::KeyPoint> match_left, 
																				std::vector<cv::KeyPoint> match_right,
																				Eigen::Matrix3d K_l,
																				Eigen::Matrix3d K_r,
																				opengv::bearingVectors_t& bearings_l, 
																				opengv::bearingVectors_t& bearings_r)
{
  double cx_l = K_l(2,0);
  double cy_l = K_l(2,1);
  double f_l  = K_l(0,0);
  double cx_r = K_r(2,0);
  double cy_r = K_r(2,1);
  double f_r  = K_r(0,0);

  double l;
  opengv::point_t pt_l, pt_r;

	for (int i = 0; i < match_left.size(); i++) 
	{
    // https://stackoverflow.com/questions/41225420/get-bearing-vector-direction-from-pixel-in-image
  	// rotation * (pixel.x - imageCentreX, pixel.y - imageCentreY, focalLength);
    // https://gist.github.com/edgarriba/10246de35799d63e0423

    pt_l.x() = match_left[i].pt.x - cx_l;
    pt_l.y() = match_left[i].pt.y - cy_l;
    pt_l.z() = f_l;
    bearings_l.push_back(pt_l / pt_l.norm());
    // ROS_INFO_STREAM("bearings_l["<< i <<"]: \n" << pt_l / l);

    pt_r.x() = match_right[i].pt.x - cx_r;
    pt_r.y() = match_right[i].pt.y - cy_r;
    pt_r.z() = f_r;
    bearings_r.push_back(pt_r / pt_r.norm());
    // ROS_INFO_STREAM("bearings_r["<< i <<"]: \n" << pt_r / l);

	} 

}


// OPENGV triangulate version
// void PointCloudManager::triangulate(std::vector<cv::KeyPoint> match_left, 
//                                     std::vector<cv::KeyPoint> match_right,
//                                     Eigen::Affine3d T_wb_left, 
// 																		Eigen::Affine3d T_clcr,
//                                     Eigen::Matrix3d K_l,
//                                     Eigen::Matrix3d K_r)
// {
//   // https://gist.github.com/edgarriba/10246de35799d63e0423
//   // https://stackoverflow.com/questions/49580137/comparing-opencv-pnp-with-opengv-pnp
//   // https://stackoverflow.com/questions/41225420/get-bearing-vector-direction-from-pixel-in-image

//   opengv::bearingVectors_t bearings_l, bearings_r;
//   // keypoint2bearing(match_left, match_right, K_l, K_r, bearings_l, bearings_r);

//   opengv::translation_t t_clcr = T_clcr.translation(); 
//   opengv::rotation_t    R_clcr = T_clcr.linear();

//   // opengv::relative_pose::CentralRelativeAdapter2 adapter;
//   // ROS_INFO_STREAM("opengv a: " << adapter.getA());


//   // opengv::relative_pose::CentralRelativeAdapter adapter(bearings_l, bearings_r, t_clcr, R_clcr);
  
// }


void PointCloudManager::mlpnp(std::vector<cv::KeyPoint> features,
															Eigen::Affine3d T_b1b2,
                              Eigen::Matrix3d K_l)
{
  double cx_l = K_l(2,0);
  double cy_l = K_l(2,1);
  double f_l  = K_l(0,0);

  opengv::point_t pt;
  opengv::points_t pts;
  for(int i = 0; i < _pc.cloud->points.size (); i ++)
  {
    pt.x() = _pc.cloud->points[i].x;
    pt.y() = _pc.cloud->points[i].y;
    pt.z() = _pc.cloud->points[i].z;
    
    pts.push_back(pt);
  }

  opengv::bearingVectors_t bearings;
  for (int i = 0; i < features.size(); i++) 
	{
    pt.x() = features[i].pt.x - cx_l;
    pt.y() = features[i].pt.y - cy_l;
    pt.z() = f_l;

    bearings.push_back(pt / pt.norm());
	} 

  opengv::translation_t t_b1b2 = T_b1b2.translation(); 
  opengv::rotation_t    R_b1b2 = T_b1b2.linear();

  opengv::cov3_mats_t covariances;

  //create a central absolute adapter
  // opengv::absolute_pose::CentralAbsoluteAdapter adapter(bearings,
  //                                                       pts,
  //                                                       t_b1b2,
  //                                                       R_b1b2,
  //                                                       covariances);

  // transformation_t mlpnp_transformation;
  // for (size_t i = 0; i < iterations; i++)
	//   mlpnp_transformation = absolute_pose::mlpnp(adapter);

}


sensor_msgs::PointCloud2 PointCloudManager::toPointCloud2Msg(std_msgs::Header header)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_bodyframe;
  for(int n = 0; n < _pc.cloud->points.size(); n++)
	{
    pcl::PointXYZ pt;
    pt.x = _pc.cloud->points[n].z;
    pt.y = _pc.cloud->points[n].x;
    pt.z = -_pc.cloud->points[n].y;

    cloud_bodyframe.points.push_back(pt);
	}

  cloud_bodyframe.width = (int)cloud_bodyframe.points.size();
  cloud_bodyframe.height = 1;
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_bodyframe, cloud_msg);

  cloud_msg.header.stamp = header.stamp;
  cloud_msg.header.frame_id = "point_cloud";

  return cloud_msg;
}