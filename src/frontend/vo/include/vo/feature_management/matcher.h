#pragma once

#include <algorithm> 
#include <array>
#include <iterator> // required for std::size

#include <boost/property_tree/ptree.hpp>

#include <opencv2/video.hpp>
#include "opencv2/core/eigen.hpp"

#ifdef OPENCV_CUDA_ENABLED
  #include <opencv2/cudafeatures2d.hpp>
#else
  #include "opencv2/features2d.hpp"
#endif

#include "support.h"

/*** Eigen packages ***/
#include <Eigen/Dense> 

enum StereoMatchMethod { DESCRIPTOR, FORWARDBACKWARD, CIRCULAR, OPTICAL_FLOW };


class Matcher
{
private:
  double match_thresh_;  // Circular match error threshold
  double reproj_thresh_; // Reprojection error threshold
  double eig_thresh_; // Reprojection error threshold
  StereoMatchMethod stereo_match_method_;
  bool display_;

  bool use_knn_procedure_;
	cv::BFMatcher desc_matcher_loop_; // for loop closure descriptor matching
  cv::Ptr<cv::DescriptorMatcher> descriptor_matcher_;

public:
  Matcher() 
  : match_thresh_(0.5), reproj_thresh_(0.5)
  {}

  Matcher(boost::property_tree::ptree config, int descriptor_norm = cv::NORM_HAMMING)
  {
    match_thresh_  = config.get< double >("matcher.match_error");
    reproj_thresh_ = config.get< double >("matcher.reprojection_error");
    eig_thresh_    = config.get< double >("matcher.LKtracker_eig_thresh");
    eig_thresh_    = config.get< double >("matcher.LKtracker_eig_thresh");

    stereo_match_method_ = StereoMatchMethod( config.get< int >("matcher.stereo_match_method") );
    use_knn_procedure_ = config.get< bool >("matcher.descriptor.knn_match");
    display_ = StereoMatchMethod( config.get< bool >("matcher.display") );
    
    desc_matcher_loop_ = cv::BFMatcher(descriptor_norm, true);

    if (config.get< std::string >("matcher.descriptor.method") == "FLANN")
      descriptor_matcher_ = cv::makePtr<cv::FlannBasedMatcher>( cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20, 10, 2)) );
      // descriptor_matcher_ = cv::FlannBasedMatcher::create();
    else if (use_knn_procedure_) // BF KNN cannot be constructed with crosscheck
      descriptor_matcher_ = cv::BFMatcher::create(descriptor_norm);
    else // BF
      descriptor_matcher_ = cv::BFMatcher::create(descriptor_norm, 
                                                  config.get< bool >("matcher.descriptor.cross_check"));
  }

  ~Matcher() 
  {
    delete descriptor_matcher_;
  }

  StereoMatchMethod getMatchMethod() {return stereo_match_method_;};

  void track(cv::Mat prev_img, 
             cv::Mat cur_img, 
             std::vector<cv::KeyPoint>& prev_kps, 
             std::vector<cv::KeyPoint>& cur_kps);

  std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> matchStereoFeatures(cv::Mat left_cur_img, 
                                                                                      cv::Mat right_cur_img, 
                                                                                      cv::Mat left_prev_img, 
                                                                                      cv::Mat right_prev_img, 
                                                                                      std::vector<cv::KeyPoint> features_left_cur, 
                                                                                      std::vector<cv::KeyPoint> features_right_cur,
                                                                                      cv::Mat desc_prev = cv::Mat(), 
                                                                                      cv::Mat desc_cur = cv::Mat());

  std::vector<cv::DMatch> matchDescriptors(cv::Mat desc_prev, 
                                           cv::Mat desc_cur, 
                                           std::vector<cv::KeyPoint>& kpts_prev, 
                                           std::vector<cv::KeyPoint>& kpts_cur);

  std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> circularMatching(cv::Mat left_cur_img, 
                                                                                   cv::Mat right_cur_img, 
                                                                                   cv::Mat left_prev_img, 
                                                                                   cv::Mat right_prev_img, 
                                                                                   std::vector<cv::KeyPoint> features_left_cur, 
                                                                                   std::vector<cv::KeyPoint> features_right_cur);

  std::vector<cv::DMatch> extractDescriptorMatches(cv::Mat desc_prev, 
                                                   cv::Mat desc_cur, 
                                                   std::vector<cv::KeyPoint>& kpts_prev, 
                                                   std::vector<cv::KeyPoint>& kpts_cur);

  std::vector<cv::DMatch> extractKNNDescriptorMatches(cv::Mat desc_prev, 
                                                      cv::Mat desc_cur, 
                                                      std::vector<cv::KeyPoint>& kpts_prev, 
                                                      std::vector<cv::KeyPoint>& kpts_cur);


  std::vector<cv::DMatch> extractDescriptorMatchesLoop(cv::Mat cur_desc, 
                                                   cv::Mat loop_desc, 
                                                   std::vector<cv::KeyPoint> cur_kpts, 
                                                   std::vector<cv::KeyPoint> loop_kpts,
                                                   std::vector<cv::Point3f> cur_landmarks,
                                                   std::vector<cv::KeyPoint>& cur_pts_matched,
                                                   std::vector<cv::KeyPoint>& loop_pts_matched,
                                                   std::vector<cv::Point3f>& cur_landmarks_matched);

  void forwardBackwardLKMatch(cv::Mat prev_img,
                              std::vector<cv::KeyPoint>& prev_kpts, 
                              cv::Mat cur_img,
                              std::vector<cv::KeyPoint>& cur_kpts);

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
    flags,
    eig_thresh_
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


std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> Matcher::matchStereoFeatures(cv::Mat left_cur_img, 
                                                                                             cv::Mat right_cur_img, 
                                                                                             cv::Mat left_prev_img, 
                                                                                             cv::Mat right_prev_img, 
                                                                                             std::vector<cv::KeyPoint> features_left_cur, 
                                                                                             std::vector<cv::KeyPoint> features_right_cur,
                                                                                             cv::Mat desc_prev, 
                                                                                             cv::Mat desc_cur)
{
  switch (stereo_match_method_)
  {
  case DESCRIPTOR:
  {
    std::vector<cv::KeyPoint> kpts_display_l = features_left_cur;
    std::vector<cv::KeyPoint> kpts_display_r = features_right_cur;
    std::vector<cv::DMatch> matches = matchDescriptors(desc_prev, 
                                                       desc_cur, 
                                                       features_left_cur, 
                                                       features_right_cur);
    
    if (display_)
      displayWindowMatches(
          left_cur_img, kpts_display_l, 
          right_cur_img, kpts_display_r,
          matches,
          "Matched Features");

    return std::make_pair(features_left_cur, features_right_cur);
  }

  case FORWARDBACKWARD:
  {
    forwardBackwardLKMatch(left_cur_img,
                           features_left_cur,
                           right_cur_img,
                           features_right_cur);
    if (display_)
      displayWindowFeatures(left_cur_img,
                            features_left_cur,
                            right_cur_img,
                            features_right_cur,
                            "Matched Features" ); 

    return std::make_pair(features_left_cur, features_right_cur);
  }
  
  case OPTICAL_FLOW:
  {
    track(left_cur_img, right_cur_img, features_left_cur, features_right_cur);

    if (display_)
      displayWindowFeatures(left_cur_img,
                            features_left_cur,
                            right_cur_img,
                            features_right_cur,
                            "Matched Features" );

    return std::make_pair(features_left_cur, features_right_cur);
  }

  case CIRCULAR:
  default:
    std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> matches = circularMatching(left_cur_img,
                            right_cur_img,
                            left_prev_img,
                            right_prev_img, 
                            features_left_cur, 
                            features_right_cur);
    if (display_)
      displayWindowFeatures(left_cur_img,
                            matches.first,
                            right_cur_img,
                            matches.second,
                            "Matched Features" );
    return matches;
  }
}                                                                                      

std::vector<cv::DMatch> Matcher::matchDescriptors(cv::Mat desc_prev, 
                                                  cv::Mat desc_cur, 
                                                  std::vector<cv::KeyPoint>& kpts_prev, 
                                                  std::vector<cv::KeyPoint>& kpts_cur)
{
  if (use_knn_procedure_) 
      return extractKNNDescriptorMatches(desc_prev, 
                                         desc_cur, 
                                         kpts_prev, 
                                         kpts_cur);
    
    else
      return extractDescriptorMatches(desc_prev, 
                                      desc_cur, 
                                      kpts_prev, 
                                      kpts_cur);
}


std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> Matcher::circularMatching(cv::Mat left_cur_img, 
                                                                                          cv::Mat right_cur_img, 
                                                                                          cv::Mat left_prev_img, 
                                                                                          cv::Mat right_prev_img, 
                                                                                          std::vector<cv::KeyPoint> features_left_cur, 
                                                                                          std::vector<cv::KeyPoint> features_right_cur)
{
  std::vector<cv::KeyPoint> matched_left, matched_right;

  if (! left_prev_img.empty())
  {
    std::vector<cv::KeyPoint> original_left = features_left_cur;
    std::vector<cv::KeyPoint> features_left_prev, features_right_prev;

    // Track in circle
    track(left_cur_img, left_prev_img, features_left_cur, features_left_prev);
    track(left_prev_img, right_prev_img, features_left_prev, features_right_prev);        
    track(right_prev_img, right_cur_img, features_right_prev, features_right_cur);
    track(right_cur_img, left_cur_img, features_right_cur, features_left_cur);

    // Ensure that features hasn't moved
    for(unsigned int i = 0; i < features_left_cur.size(); i++)
    {
      for (cv::KeyPoint& original_feature : original_left)
      {
        if (features_left_cur[i].class_id != original_feature.class_id)
          continue;

        if ( pow((features_left_cur[i].pt.x - original_feature.pt.x), 2) 
           + pow((features_left_cur[i].pt.y - original_feature.pt.y), 2) < pow(match_thresh_, 2) )
        {
          matched_left.push_back(features_left_cur[i]);
          matched_right.push_back(features_right_cur[i]);

          break;
        }
      }
    }
  }
  else // Previous frame is empty
  {
    matched_left = features_left_cur;
    forwardBackwardLKMatch(left_cur_img,
                           matched_left,
                           right_cur_img,
                           matched_right);
  }
  return std::make_pair(matched_left, matched_right);
}



std::vector<cv::DMatch> Matcher::extractDescriptorMatches(cv::Mat desc_prev, 
                                                          cv::Mat desc_cur, 
                                                          std::vector<cv::KeyPoint>& kpts_prev, 
                                                          std::vector<cv::KeyPoint>& kpts_cur)
{
  std::vector<cv::DMatch> matches, matches_filtered;
  descriptor_matcher_->match(desc_prev, desc_cur, matches);
  
  // Only keep matched points 
  std::vector<cv::KeyPoint> kpts_prev_matched, kpts_cur_matched;
  for (int i = 0; i < matches.size(); i++)
  {
    int prev_idx = matches[i].queryIdx;
    int cur_idx = matches[i].trainIdx;
    
    if ( std::abs(kpts_prev[prev_idx].pt.y - kpts_cur[cur_idx].pt.y) < 50 )
    {
      cv::KeyPoint pt_r = kpts_cur[cur_idx];
      pt_r.class_id = kpts_prev[prev_idx].class_id;
      kpts_prev_matched.push_back(kpts_prev[prev_idx]);
      kpts_cur_matched.push_back(pt_r);
      matches_filtered.push_back(matches[i]);
    }
  }

  kpts_prev = kpts_prev_matched;
  kpts_cur = kpts_cur_matched;

  return matches_filtered;
}


std::vector<cv::DMatch> Matcher::extractKNNDescriptorMatches(cv::Mat desc_prev, 
                                                             cv::Mat desc_cur, 
                                                             std::vector<cv::KeyPoint>& kpts_prev, 
                                                             std::vector<cv::KeyPoint>& kpts_cur)
{
  std::vector<std::vector<cv::DMatch>> matches;
  descriptor_matcher_->knnMatch(desc_prev, desc_cur, matches, 2);
  
  // Only keep matched points 
  std::vector<cv::DMatch> ret_matches;
  std::vector<cv::KeyPoint> kpts_prev_matched, kpts_cur_matched;
  for (const std::vector<cv::DMatch> match : matches)
  {
    if (match[0].distance < match[1].distance * 0.7)
    {
      int prev_idx = match[0].queryIdx;
      int cur_idx = match[0].trainIdx;
      
      // if ( std::abs(kpts_prev[prev_idx].pt.y - kpts_cur[cur_idx].pt.y) < 100 )
      {
        cv::KeyPoint pt_r = kpts_cur[cur_idx];
        pt_r.class_id = kpts_prev[prev_idx].class_id;
        kpts_prev_matched.push_back(kpts_prev[prev_idx]);
        kpts_cur_matched.push_back(pt_r);
        ret_matches.push_back(match[0]);;
      }
    }

  }

  kpts_prev = kpts_prev_matched;
  kpts_cur = kpts_cur_matched;

  return ret_matches;
}


std::vector<cv::DMatch> Matcher::extractDescriptorMatchesLoop(cv::Mat cur_desc, 
                                                          cv::Mat loop_desc, 
                                                          std::vector<cv::KeyPoint> cur_kpts, 
                                                          std::vector<cv::KeyPoint> loop_kpts,
                                                          std::vector<cv::Point3f> cur_landmarks,
                                                          std::vector<cv::KeyPoint>& cur_pts_matched,
                                                          std::vector<cv::KeyPoint>& loop_pts_matched,
                                                          std::vector<cv::Point3f>& cur_landmarks_matched)
{
  std::vector<cv::DMatch> matches, matches_filtered;
  desc_matcher_loop_.match(cur_desc, loop_desc, matches);
  
  // Only keep matched points 
  for (int i = 0; i < matches.size(); i++)
  {
    int cur_id = matches[i].queryIdx;
    int loop_id = matches[i].trainIdx;
    
    if ( std::abs(cur_kpts[cur_id].pt.y - loop_kpts[loop_id].pt.y) < 5 )
    {
      ROS_INFO("added");
      cur_pts_matched.push_back(cur_kpts[cur_id]);
      loop_pts_matched.push_back(loop_kpts[loop_id]);
      cur_landmarks_matched.push_back(cur_landmarks[cur_id]);
      matches_filtered.push_back(matches[i]);
    }
  }

  return matches_filtered;
}



void Matcher::forwardBackwardLKMatch(cv::Mat prev_img,
                                     std::vector<cv::KeyPoint>& prev_kpts, 
                                     cv::Mat cur_img,
                                     std::vector<cv::KeyPoint>& cur_kpts)
{
  std::vector<cv::KeyPoint> prev_tracked, cur_tracked, prev_matches, cur_matches;
  track(prev_img, cur_img, prev_kpts, cur_tracked);
  track(cur_img, prev_img, cur_tracked, prev_tracked);

  for(unsigned int i = 0; i < prev_tracked.size(); i++)
  {
    for(unsigned int j = 0; j < prev_kpts.size(); j++)
    {
      if (prev_tracked[i].class_id != prev_kpts[j].class_id)
        continue;

      if ( pow((prev_tracked[i].pt.x - prev_kpts[j].pt.x), 2) 
          + pow((prev_tracked[i].pt.y - prev_kpts[j].pt.y), 2) < pow(match_thresh_, 2) )
      {
        prev_matches.push_back(prev_tracked[i]);
        cur_matches.push_back(cur_tracked[i]);

        break;
      }
    }
  }

  prev_kpts = prev_matches;
  cur_kpts = cur_matches;
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

  // std::vector<cv::Point2f> point2D_left, point2D_right;
  // cv::KeyPoint::convert(match_left, point2D_left);
  // cv::KeyPoint::convert(match_right, point2D_right);

  // cv::Mat point3D_homogenous(4, point2D_left.size(), CV_64FC1);                                                 // https://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints
  // cv::triangulatePoints(P_l, P_r, point2D_left, point2D_right, point3D_homogenous); // https://gist.github.com/cashiwamochi/8ac3f8bab9bf00e247a01f63075fedeb


  int n_err = 0;
  int num_features = match_left.size();
  for(int i = 0; i < num_features; i++)
  {
    int pt_it = i - n_err;

    // Triangulate
    cv::Mat pt3D = DLT(match_left[pt_it].pt, match_right[pt_it].pt, P_l, P_r);

    // cv::Mat pt3Dh = point3D_homogenous.col(i); 
    // cv::Mat pt3Dc = pt3Dh.rowRange(0,3)/pt3Dh.at<float>(3); // / Homogenous to cartesian
    // cv::Mat pt3D;
    // pt3Dc.convertTo(pt3D, CV_64FC1);

    // Check potential errors when triangulating
    cv::Point2f proj_l = project(pt3D, P_l);
    cv::Point2f proj_r = project(pt3D, P_r);

    bool positive = pt3D.at<double>(2) < 0;
    bool less_than_200 = pt3D.at<double>(2) > 200;
    bool same_id = match_left[i].class_id != match_right[i].class_id;

    // std::cout << "\n-----" << std::endl;
    // std::cout << "pt3D[" << i << "] : " << pt3D.t() << std::endl;
    // std::cout << "- z < 0: " << positive << std::endl;
    // std::cout << "- z > 200: " << less_than_200 << std::endl;
    // std::cout << "\nLeft  pt org: " << match_left[pt_it].pt << ", reprojected: " << proj_l << std::endl;
    // std::cout << "- Reproj left error: " << std::sqrt(pow((match_left[pt_it].pt.x - proj_l.x), 2)  + pow((match_left[pt_it].pt.y - proj_l.y), 2)) << std::endl;
    // std::cout << "- Left x diff: " << (match_left[pt_it].pt.x - proj_l.x) << std::endl;
    // std::cout << "- Left y diff: " << (match_left[pt_it].pt.y - proj_l.y) << std::endl;
    // std::cout << "\nRight pt org: " << match_right[pt_it].pt << ", reprojected: " << proj_r << std::endl;
    // std::cout << "- Reproj right error: " << std::sqrt(pow((match_right[pt_it].pt.x - proj_r.x), 2) + pow((match_right[pt_it].pt.y - proj_r.y), 2)) << std::endl;
    // std::cout << "- Right x diff: " << (match_right[pt_it].pt.x - proj_r.x) << std::endl;
    // std::cout << "- Right y diff: " << (match_right[pt_it].pt.y - proj_r.y) << std::endl;

    if ( (pt3D.at<double>(2) < 0)                              // Point is not in front of camera
      || (pt3D.at<double>(2) > 200)                           // Point is more than ..m away from camera
      || (match_left[i].class_id != match_right[i].class_id)   // Not matched
      || ( std::sqrt( pow((match_left[pt_it].pt.x - proj_l.x), 2)  + pow((match_left[pt_it].pt.y - proj_l.y), 2)  ) > std::sqrt(reproj_thresh_) ) // To large reprojection error in left cam
      || ( std::sqrt( pow((match_right[pt_it].pt.x - proj_r.x), 2) + pow((match_right[pt_it].pt.y - proj_r.y), 2) ) > std::sqrt(reproj_thresh_) ) // To large reprojection error in right cam
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
  // ROS_INFO_STREAM("triangulation() - Correct: " << wrld_pts.size() << ", Erronous: " << n_err);


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

