#pragma once

// #include "gtsam_pose_estimator.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"

#include "gtsam/geometry/SimpleCamera.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"

#include "motionBA/resectioning.h"
#include "motionBA/pose_estimate.h"


/// \brief Iterative pose estimator for calibrated camera with 3D-2D correspondences.
/// This pose estimator need another pose estimator,
/// which it will use to initialize estimate and find inliers.
class MotionBundleAdjuster // : public PoseEstimator
{
private:
  Eigen::Matrix3d K_;
  gtsam::Cal3_S2::shared_ptr K_gtsam_;

public:
  /// \brief Constructs pose estimator.
  /// \param initial_pose_estimator Pointer to a pose estimator for initialization and inlier extraction.
  /// \param K Camera calibration matrix.
  MotionBundleAdjuster(const Eigen::Matrix3d& K) 
  : K_{K}, K_gtsam_(new gtsam::Cal3_S2(K_(0,0), K_(1,1), K_(0,1), K_(0,2), K_(1,2)))
  {}


  /// \brief Estimates camera pose from 3D-2D correspondences.
  /// \param image_points 2D image points.
  /// \param world_points 3D planar world points.
  /// \return The results. Check PoseEstimate::isFound() to check if solution was found.
  PoseEstimate estimate(PoseEstimate init_estimate);
};



PoseEstimate MotionBundleAdjuster::estimate(PoseEstimate init_estimate) 
{
  // Create factor graph.
  gtsam::NonlinearFactorGraph graph;

  // Use same robust measurement noise model.
  // Specify assumed pixel noise below.
  const double pixel_std_dev = 0.5;
  gtsam::SharedNoiseModel measurementNoise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345),
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(pixel_std_dev, pixel_std_dev)));

  // Add each 3D-2D correspondence to the factor graph.
  for (size_t i=0; i<init_estimate.image_inlier_points.size(); ++i)
  {
    const auto& img_pt = init_estimate.image_inlier_points[i];
    const auto& wrld_pt = init_estimate.world_inlier_points[i];

    graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), K_gtsam_,
                                            gtsam::Point2(img_pt.x, img_pt.y),
                                            gtsam::Point3(wrld_pt.x, wrld_pt.y, wrld_pt.z));
  }

  // Set initial estimate.
  Eigen::Affine3d init_pose = init_estimate.T_wb;
  gtsam::Values initial;
  initial.insert(X(1), gtsam::Pose3(gtsam::Rot3(init_pose.linear()), init_pose.translation()));

  // Find the optimal camera pose given correspondences and assumed noise model.
  gtsam::Values result;
  try
  {
    // result = gtsam::GaussNewtonOptimizer(graph, initial).optimize(); 
    result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  }
  catch (gtsam::CheiralityException& e)
  {
    return {};
  }

  // Update pose estimate.
  init_estimate.T_wb = Eigen::Affine3d{result.at<gtsam::Pose3>(X(1)).matrix()};
  return init_estimate;
}