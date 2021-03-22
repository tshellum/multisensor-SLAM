#pragma once

#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"

#include "gtsam/geometry/SimpleCamera.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"

#include "objective_functions/resectioning_pose.h"


namespace BA
{

/// \brief Iterative pose estimator for calibrated camera with 3D-2D correspondences.
/// This pose estimator need another pose estimator,
/// which it will use to initialize estimate and find inliers.
class MotionEstimator
{
private:
  Eigen::Matrix3d K_;
  gtsam::Cal3_S2::shared_ptr K_gtsam_;

  // Use same robust measurement noise model.
  const gtsam::SharedNoiseModel feature_noise_; 

public:
  /// \brief Constructs pose estimator.
  /// \param K Camera calibration matrix.
  /// \param pixel_std_dev Measurement noise.
  MotionEstimator(const Eigen::Matrix3d& K,
                  const double pixel_std_dev = 0.5) 
  : K_{K}, K_gtsam_(new gtsam::Cal3_S2(K_(0,0), K_(1,1), K_(0,1), K_(0,2), K_(1,2)))
  , feature_noise_( gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345),
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(pixel_std_dev, pixel_std_dev))
      )
    )
  {}

  /// \brief Estimates camera pose from 3D-2D correspondences.
  /// \param T_r Initial estimate of relative transformation.
  /// \param image_points 2D image points.
  /// \param world_points 3D planar world points.
  /// \return The results.
  Eigen::Affine3d estimate(const Eigen::Affine3d T_r,                           
                           const std::vector<cv::Point3f> world_inlier_points,
                           const std::vector<cv::Point2f> image_inlier_points);
};


Eigen::Affine3d MotionEstimator::estimate(const Eigen::Affine3d T_r,                           /// Camera pose in the world.                                        
                                          const std::vector<cv::Point3f> world_inlier_points,  /// 3D inlier world points 
                                          const std::vector<cv::Point2f> image_inlier_points)  /// 2D inlier image points
{
  // Create factor graph.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  gtsam::Values result;

  // Add each 3D-2D correspondence to the factor graph.
  for (size_t i=0; i<image_inlier_points.size(); ++i)
  {
    const cv::Point2f img_pt  = image_inlier_points[i];
    const cv::Point3f wrld_pt = world_inlier_points[i];

    graph.emplace_shared<ResectioningPoseFactor>(feature_noise_, X(1), K_gtsam_,
                                                 gtsam::Point2(img_pt.x, img_pt.y),
                                                 gtsam::Point3(wrld_pt.x, wrld_pt.y, wrld_pt.z));
  }

  // Set initial estimate.
  initial.insert( X(1), gtsam::Pose3(T_r.matrix()) );

  // initial.print("Initial Estimates:\n");

  // Find the optimal camera pose given correspondences and assumed noise model.
  try
  {
    // result = gtsam::GaussNewtonOptimizer(graph, initial).optimize(); 
    result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  }
  catch (gtsam::CheiralityException& e)
  {
    return Eigen::Affine3d::Identity();
  }

  // result.print("Optimized results:\n");

  // Update pose estimate.
  return Eigen::Affine3d{result.at<gtsam::Pose3>(X(1)).matrix()};
}


} // namespace BA