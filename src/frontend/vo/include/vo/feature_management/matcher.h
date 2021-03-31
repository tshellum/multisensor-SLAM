#pragma once

#include <algorithm> 
#include <array>
#include <iterator> // required for std::size

#include <boost/property_tree/ptree.hpp>

#include <opencv2/video.hpp>
#include "opencv2/features2d.hpp"

#include "support.h"

/*** PCL packages ***/
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/*** Eigen packages ***/
#include <Eigen/Dense> 


class Matcher
{
private:
  double match_err_;  // Circular match error threshold
  double reproj_err_; // Reprojection error threshold

public:
  Matcher() 
  : match_err_(0.5), reproj_err_(0.5)
  {}

  Matcher(boost::property_tree::ptree config)
  {
    match_err_  = config.get< double >("matcher.match_error");
    reproj_err_ = config.get< double >("matcher.reprojection_error");;
  }

  ~Matcher() {}


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

  cv::Point2f project(cv::Mat pt3D, 
                      cv::Mat P);

  std::pair<std::vector<cv::Point3f>, std::vector<int>> triangulate(std::vector<cv::KeyPoint>& match_left, 
                                                                    std::vector<cv::KeyPoint>& match_right,
                                                                    cv::Mat P_l,
                                                                    cv::Mat P_r);

  std::vector<cv::KeyPoint> projectLandmarks(std::vector<cv::Point3f> landmarks, 
                                             Eigen::Affine3d T,
                                             Eigen::Matrix3d K,
                                             std::vector<cv::KeyPoint>& kpts_prev);
};



void Matcher::track(cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::KeyPoint>& prev_kps, std::vector<cv::KeyPoint>& cur_kps)
{
  if (prev_img.empty() || prev_kps.empty())
    return;
  
  // Point vectors
  std::size_t n_pts = prev_kps.size(); 
  std::vector<cv::Point2f> prev_pts(n_pts), cur_pts(n_pts);

  // for (std::size_t i = 0; i < n_pts; ++i)
  //   prev_pts[i] = prev_kps[i].pt; 

  cv::KeyPoint::convert(prev_kps, prev_pts);
  cv::KeyPoint::convert(cur_kps, cur_pts);

  std::vector<uchar> status; 
  std::vector<float> error; 

  // Default values are specified so that the cv::OPTFLOW_USE_INITIAL_FLOW flag may be used
  cv::Size winSize = cv::Size(21, 21);
  int maxPyramidLevel = 3;
  cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.01);
  
  int flags = 0;
  if ( prev_kps.size() == cur_kps.size() )
    flags = cv::OPTFLOW_USE_INITIAL_FLOW;

  // https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
  cv::calcOpticalFlowPyrLK(
    prev_img, 
    cur_img, 
    prev_pts, 
    cur_pts, 
    status, 
    error,
    winSize,
    maxPyramidLevel,
    criteria,
    flags
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



std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> Matcher::circularMatching(cv::Mat left_cur_img, 
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

    // track(left_cur_img, right_cur_img, features_left_cur, features_right_cur);
    // track(right_cur_img, right_prev_img, features_right_cur, features_right_prev);        
    // track(right_prev_img, left_prev_img, features_right_prev, features_left_prev);
    // track(left_prev_img, left_cur_img, features_left_prev, features_left_cur);

    // Ensure that features hasn't moved
    int match_nr = 1;
    for(unsigned int i = 0; i < features_left_cur.size(); i++)
    {
      for (cv::KeyPoint& original_feature : original_left)
      {
        if (features_left_cur[i].class_id != original_feature.class_id)
          continue;

        if ( pow((features_left_cur[i].pt.x - original_feature.pt.x), 2) 
           + pow((features_left_cur[i].pt.y - original_feature.pt.y), 2) < pow(match_err_, 2) )
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
cv::Mat Matcher::DLT(cv::Point2f pt_l, 
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


cv::Point2f Matcher::project(cv::Mat pt3D, cv::Mat P)
{
  cv::Mat x_homogenous = cv::Mat::ones(4, 1, pt3D.type());
  pt3D.copyTo(x_homogenous(cv::Rect(0, 0, 1, 3)));

  cv::Mat pt_proj = P * x_homogenous;
  return cv::Point2f(pt_proj.at<double>(0) / pt_proj.at<double>(2),
                     pt_proj.at<double>(1) / pt_proj.at<double>(2));
}



std::pair<std::vector<cv::Point3f>, std::vector<int>> Matcher::triangulate(std::vector<cv::KeyPoint>& match_left, 
                                                                           std::vector<cv::KeyPoint>& match_right,
                                                                           cv::Mat P_l,
                                                                           cv::Mat P_r)
{  
  std::vector<cv::Point3f> wrld_pts;
  std::vector<int> indices;

  if (match_left.empty() || match_right.empty())
    return std::make_pair(wrld_pts, indices);;

  int n_err = 0;
  int num_features = match_left.size();
  for(int i = 0; i < num_features; i++)
  {
    int pt_it = i - n_err;

    // Triangulate
    cv::Mat pt3D = DLT(match_left[pt_it].pt, match_right[pt_it].pt, P_l, P_r);

    // Check potential errors when triangulating
    cv::Point2f proj_l = project(pt3D, P_l);
    cv::Point2f proj_r = project(pt3D, P_r);

    if ( (pt3D.at<double>(2) < 0)                              // Point is not in front of camera
      || (pt3D.at<double>(2) > 100)                           // Point is more than ..m away from camera
      || (match_left[i].class_id != match_right[i].class_id)   // Not matched
      || ( pow((match_left[pt_it].pt.x - proj_l.x), 2)  + pow((match_left[pt_it].pt.y - proj_l.y), 2)  > reproj_err_ ) // To large reprojection error in left cam
      || ( pow((match_right[pt_it].pt.x - proj_r.x), 2) + pow((match_right[pt_it].pt.y - proj_r.y), 2) > reproj_err_ ) // To large reprojection error in right cam
    )
    {
      match_left.erase(match_left.begin() + pt_it);
      match_right.erase(match_right.begin() + pt_it);
      n_err++;
      continue;
    }

    // Safe --> add point
    cv::Point3f pt;
    pt.x = pt3D.at<double>(0);
    pt.y = pt3D.at<double>(1);
    pt.z = pt3D.at<double>(2);

    wrld_pts.push_back(pt);
    indices.push_back(match_left[pt_it].class_id);
    
  }
  ROS_INFO_STREAM("triangulation() - Correct: " << wrld_pts.size() << ", Erronous: " << n_err);


  return std::make_pair(wrld_pts, indices);;
}



std::vector<cv::KeyPoint> Matcher::projectLandmarks(std::vector<cv::Point3f> landmarks, 
                                                    Eigen::Affine3d T,
                                                    Eigen::Matrix3d K,
                                                    std::vector<cv::KeyPoint>& kpts_prev)
{
  std::vector<cv::KeyPoint> kpts_prev_cp = kpts_prev;

  int width = 2*K(0,2);
  int height = 2*K(1,2);

  // Create projection matrix
  cv::Mat R, t, Rt, K_cv;
  cv::eigen2cv(Eigen::Matrix3d{ T.linear() }, R);
  cv::eigen2cv(Eigen::Vector3d{ T.translation() }, t);
  cv::eigen2cv(K, K_cv);
  
  cv::hconcat(R, t, Rt);
  cv::Mat P = K_cv*Rt;

  // Project landmarks
  std::vector<cv::KeyPoint> projected_pts;
  int n_err = 0;
  int num_features = landmarks.size();
  for(int i = 0; i < num_features; i++)
  {
    int pt_it = i - n_err;

    cv::Mat pt3D = cv::Mat::zeros(3,1, CV_64F);
    pt3D.at<double>(0) = landmarks[i].x;
    pt3D.at<double>(1) = landmarks[i].y;
    pt3D.at<double>(2) = landmarks[i].z;

    // Check potential errors when triangulating
    cv::Point2f proj = project(pt3D, P);

    if ( proj.x < 0
      || proj.y < 0
      || proj.x > width
      || proj.y > height )
    {
      kpts_prev.erase(kpts_prev.begin() + pt_it);
      n_err++;
      continue;
    }

    cv::KeyPoint kp(proj, 1.f);
    projected_pts.push_back(kp);
  }

  // Too many features are lost 
  if ( kpts_prev.size() < (kpts_prev_cp.size() / 2) )
    kpts_prev = kpts_prev_cp;

  return projected_pts;
}

