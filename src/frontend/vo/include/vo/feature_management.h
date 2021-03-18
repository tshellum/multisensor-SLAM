#pragma once

#include <algorithm> 
#include <array>
#include <iterator> // required for std::size

#include <opencv2/video.hpp>
#include "opencv2/features2d.hpp"

#include "support.h"


/*** PCL packages ***/
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class FeatureManager
{
private:
  // Detector
  cv::Ptr<cv::Feature2D> extractor_;
  cv::Ptr<cv::Feature2D> descriptor_; 

  // Function parameters
  int grid_size_;
  int width_, height_;
  int patch_w_, patch_h_;
  int n_buckets_;
  int max_features_;
  int nms_distance_;

  // Point parameters    
  int n_id_;

public:
  FeatureManager(ros::NodeHandle nh) 
  : n_id_(0)
  {
    // Detector parameters
    int threshold;
    nh.getParam("/detector/feature_threshold", threshold);
    nh.getParam("/detector/max_number_of_features", max_features_);
    nh.getParam("/detector/grid_size", grid_size_);
    nh.getParam("/detector/nms_distance", nms_distance_);
    
    std::string extractor_type, descriptor_type;
    nh.getParam("/detector/extractor_type", extractor_type);
    nh.getParam("/detector/descriptor_type", descriptor_type);

    int img_width_full, img_height_full;
    nh.getParam("/camera_left/image_width", img_width_full);
    nh.getParam("/camera_left/image_height", img_height_full);

    // Extractor
    if (extractor_type == "GFFT")
      extractor_ = cv::GFTTDetector::create(max_features_, // maximum number of features
                                            0.01,         // quality level
                                            10);          // minimum allowed distance
    else if (extractor_type == "FAST")
      extractor_ = cv::FastFeatureDetector::create(threshold, // threshold
                                                     true);      // NMS
    else if (extractor_type == "ORB")
      extractor_ = cv::ORB::create();
    else
      extractor_ = cv::FastFeatureDetector::create();
                      

    // Descriptor                          
    if (descriptor_type == "ORB")
      descriptor_ = cv::ORB::create(max_features_);
    else 
      descriptor_ = cv::ORB::create();
  

    width_ = img_width_full - (img_width_full % grid_size_);
    height_ = img_height_full - (img_height_full % grid_size_);

    patch_w_ = width_ / grid_size_;
    patch_h_ = height_ / grid_size_;

    n_buckets_ = grid_size_*grid_size_;
  }

  ~FeatureManager() {}

  void bucketedFeatureDetection(cv::Mat image, 
                                std::vector<cv::KeyPoint>& features);
  
  void track(cv::Mat prev_img, 
             cv::Mat cur_img, 
             std::vector<cv::KeyPoint>& prev_kps, 
             std::vector<cv::KeyPoint>& cur_kps);

  std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> circularMatching(cv::Mat left_cur_img, 
                                                                                   cv::Mat right_cur_img, 
                                                                                   cv::Mat left_prev_img, 
                                                                                   cv::Mat right_prev_img, 
                                                                                   std::vector<cv::KeyPoint> features_left_cur, 
                                                                                   std::vector<cv::KeyPoint> features_right_cur);

  cv::Mat DLT(cv::Point2f pt_l, 
                  cv::Point2f pt_r,
                  cv::Mat P_l,
                  cv::Mat P_r);

  pcl::PointCloud<pcl::PointXYZ>::Ptr triangulate(std::vector<cv::KeyPoint> match_left, 
                                                  std::vector<cv::KeyPoint> match_right,
                                                  cv::Mat P_cl,
                                                  cv::Mat P_cr);

};


void FeatureManager::bucketedFeatureDetection(cv::Mat image, std::vector<cv::KeyPoint>& features)
{
  // Place tracked features from previous frame in buckets
  std::vector<cv::KeyPoint> feature_buckets[n_buckets_];
  for (cv::KeyPoint& kpt : features)
    feature_buckets[int(kpt.pt.x / patch_w_) * grid_size_ + int(kpt.pt.y / patch_h_)].push_back(kpt);
  
  
  // Detect new features w/ NMS
  std::vector<cv::KeyPoint> patch_features; 
  int it_bucket = 0;

  for (unsigned int x = 0; x < width_; x += patch_w_)
  {
    for (unsigned int y = 0; y < height_; y += patch_h_)
    {
      // Detect features
      cv::Mat mask = cv::Mat();
      cv::Mat img_patch_cur = image(cv::Rect(x, y, patch_w_, patch_h_)).clone();
      
      extractor_->detect(img_patch_cur, patch_features, mask); 

      // Sort to retain strongest features
      std::sort(patch_features.begin(), patch_features.end(), 
          [](const cv::KeyPoint& kpi, const cv::KeyPoint& kpj){ return kpi.response > kpj.response;  }
      ); 

      // Retain tracked features and only add strongest features
      int num_bucket_features = 0;
      for (cv::KeyPoint& new_feature : patch_features)
      { 
        // Not more that max_features number of features in each bucket 
        if ( (feature_buckets[it_bucket].size() + num_bucket_features) > max_features_ )
          break;

        // Correct for patch position
        new_feature.pt.x += x;
        new_feature.pt.y += y; 

        int x0 = new_feature.pt.x;
        int y0 = new_feature.pt.y;

        bool skip = false;
        for (cv::KeyPoint& existing_feature : feature_buckets[it_bucket]) 
        {
          // NMS: Skip point if point is in vicinity of existing points
          if ( pow((existing_feature.pt.x - x0), 2) + pow((existing_feature.pt.y - y0), 2) < pow(nms_distance_, 2) )
          {
            skip = true;
            break;
          }                        
        }

        if (skip)
            continue;

        new_feature.class_id = n_id_++; 

        features.push_back(new_feature);
        num_bucket_features++;
      }

      it_bucket++;
    }
  }
}



void FeatureManager::track(cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::KeyPoint>& prev_kps, std::vector<cv::KeyPoint>& cur_kps)
{
  if (prev_img.empty() || prev_kps.empty())
    return;
  
  // Point vectors
  std::size_t n_pts = prev_kps.size(); 
  std::vector<cv::Point2f> prev_pts(n_pts), cur_pts(n_pts);

  for (std::size_t i = 0; i < n_pts; ++i)
    prev_pts[i] = prev_kps[i].pt; 

  std::vector<uchar> status; 
  std::vector<float> error; 

  // https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
  cv::calcOpticalFlowPyrLK(
    prev_img, 
    cur_img, 
    prev_pts, 
    cur_pts, 
    status, 
    error
  );

  std::vector<cv::KeyPoint> tracked_kpts_prev, tracked_kpts_cur;
  for(uint i = 0; i < n_pts; ++i)
  {
    // Select good points - Status variable not enough..
    if (
      (status[i] == 1) 
      && (cur_pts[i].x > 0) && (cur_pts[i].y > 0)
      && (cur_pts[i].x < prev_img.cols) && (cur_pts[i].y < prev_img.rows)
    ) 
    {
      tracked_kpts_prev.push_back(
        cv::KeyPoint
        (
          prev_pts[i], 
          prev_kps[i].size, 
          prev_kps[i].angle, 
          prev_kps[i].response, 
          prev_kps[i].octave, 
          prev_kps[i].class_id
        )
      );
      
      tracked_kpts_cur.push_back(
        cv::KeyPoint
        (
          cur_pts[i], 
          prev_kps[i].size, 
          prev_kps[i].angle, 
          prev_kps[i].response, 
          prev_kps[i].octave, 
          prev_kps[i].class_id
        )
      );
    }
  }
  prev_kps = tracked_kpts_prev;
  cur_kps = tracked_kpts_cur;
}



std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> FeatureManager::circularMatching(cv::Mat left_cur_img, 
                                                                                                 cv::Mat right_cur_img, 
                                                                                                 cv::Mat left_prev_img, 
                                                                                                 cv::Mat right_prev_img, 
                                                                                                 std::vector<cv::KeyPoint> features_left_cur, 
                                                                                                 std::vector<cv::KeyPoint> features_right_cur)
{
  std::vector<cv::KeyPoint> matched_left, matched_right;
  std::vector<cv::KeyPoint> original_left = features_left_cur;

  if (! left_prev_img.empty())
  {
    std::vector<cv::KeyPoint> features_left_prev, features_right_prev;

    // Track in circle
    track(left_cur_img, left_prev_img, features_left_cur, features_left_prev);
    track(left_prev_img, right_prev_img, features_left_prev, features_right_prev);        
    track(right_prev_img, right_cur_img, features_right_prev, features_right_cur);
    track(right_cur_img, left_cur_img, features_right_cur, features_left_cur);

    // Ensure that feature hasn't moved
    int match_nr = 1;
    for(unsigned int i = 0; i < features_left_cur.size(); i++)
    {
      for (cv::KeyPoint& original_feature : original_left)
      {
        if (features_left_cur[i].class_id != original_feature.class_id)
          continue;

        if ( pow((features_left_cur[i].pt.x - original_feature.pt.x), 2) 
           + pow((features_left_cur[i].pt.y - original_feature.pt.y), 2) < pow(1, 2) )
        {
          matched_left.push_back(features_left_cur[i]);
          matched_right.push_back(features_right_cur[i]);

          break;
        }
      }
      match_nr++;
    }
  }
  return std::make_pair(matched_left, matched_right);
}



// https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/CameraModels/KannalaBrandt8.cpp
cv::Mat FeatureManager::DLT(cv::Point2f pt_l, 
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



pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureManager::triangulate(std::vector<cv::KeyPoint> match_left, 
                                                                std::vector<cv::KeyPoint> match_right,
                                                                cv::Mat P_l,
                                                                cv::Mat P_r)
{
  ROS_INFO_STREAM("begin - match_left.size(): " << match_right.size() << ", match_left.size(): " << match_right.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  if (match_left.empty())
    return cloud;
        
  std::vector<cv::Point2f> point2D_left, point2D_right;
  cv::KeyPoint::convert(match_left, point2D_left);
  cv::KeyPoint::convert(match_right, point2D_right);

  // cv::Mat point3D_homogenous(4, point2D_left.size(), CV_64FC1);                                                 // https://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints
  // cv::triangulatePoints(P_cl, P_cr, point2D_left, point2D_right, point3D_homogenous); // https://gist.github.com/cashiwamochi/8ac3f8bab9bf00e247a01f63075fedeb

  int n_err = 0;
  for(int i = 0; i < point2D_left.size(); i++)
  {
    ROS_INFO("convert: ");

    // cv::convertPointsFromHomogeneous(point3D_homogenous.col(i).t(), pt3D);
    cv::Mat pt3D = DLT(point2D_left[i], point2D_right[i], P_l, P_r);

    // cv::Mat pt3Dh = point3D_homogenous.col(i); 
    // cv::Mat pt3Dc = pt3Dh.rowRange(0,3)/pt3Dh.at<float>(3); // / Homogenous to cartesian
    // cv::Mat pt3D; 
    // pt3Dc.convertTo(pt3D, CV_64FC1);

    if (pt3D.at<double>(2) > 0)
    {
      ROS_INFO("pcl::PointXYZ pt: ");

      pcl::PointXYZ pt;
      pt.x = pt3D.at<double>(0);
      pt.y = pt3D.at<double>(1);
      pt.z = pt3D.at<double>(2);

      ROS_INFO_STREAM("pt: " << pt);

      cloud->points.push_back(pt);
    }
    else
    {
      n_err++;
    }
  }
  ROS_INFO("rosinfo ");

  ROS_INFO_STREAM("triangulation() - Correct: " << cloud->points.size() << ", Erronous: " << n_err);

  return cloud;
}