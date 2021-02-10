#pragma once

#include "opencv2/core.hpp"
#include "sophus/se3.hpp"
#include <memory>

// #include <Eigen/Geometry> 
// #include <Eigen/Dense>

/// \brief Struct for 3D-2D pose estimation results
struct PoseEstimate
{
  Sophus::SE3d pose_W_C;                        /// Camera pose in the world.
  Eigen::Affine3d T_wb;                        /// Camera pose in the world.
  std::vector<cv::Point2f> image_inlier_points; /// 2D inlier image points.
  std::vector<cv::Point3f> world_inlier_points; /// 3D inlier world points.

  PoseEstimate() {};

  PoseEstimate(std::vector<cv::Point2f> image_points, std::vector<cv::Point3f> world_points)
  : image_inlier_points(image_points), world_inlier_points(world_points) 
  {};

  // PoseEstimate(std::vector<cv::Point2f> image_points, std::vector<cv::Point3f> world_points, Sophus::SE3d pose)
  // : image_inlier_points(image_points), world_inlier_points(world_points), pose_W_C(pose) 
  // {};

  PoseEstimate(std::vector<cv::Point2f> image_points, std::vector<cv::Point3f> world_points, cv::Mat T)
  : image_inlier_points(image_points), world_inlier_points(world_points) 
  {
    cv::Mat R = T(cv::Rect(0, 0, 3, 3));
    cv::Mat t = T(cv::Rect(3, 0, 1, 3));
    
    Eigen::Matrix3d R_wb;
    Eigen::Vector3d t_wb;
    cv::cv2eigen(R, R_wb);
    cv::cv2eigen(t, t_wb);
    
    T_wb.linear() = R_wb;
    T_wb.translation() = t_wb;
  };

  /// \brief Checks if estimation succeeded.
  /// \return True if result was found.
  bool isFound() const
  {
    // Default identity orientation means looking away,
    // therefore using default value when no valid estimate was found.
    return !pose_W_C.rotationMatrix().isIdentity(1e-8);
  }
};