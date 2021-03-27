#pragma once

#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include "gtsam/geometry/SimpleCamera.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"

#include "objective_functions/resectioning.h"


namespace BA
{

/// \brief Iterative pose estimator for calibrated camera with 3D-2D correspondences.
/// This pose estimator need another pose estimator,
/// which it will use to initialize estimate and find inliers.
class MotionEstimator
{
private:
  gtsam::Cal3_S2::shared_ptr K_;
  gtsam::Cal3_S2Stereo::shared_ptr K_stereo_;

  gtsam::Pose3 T_wb0_;

  // Use same robust measurement noise model.
  const gtsam::SharedNoiseModel feature_noise_;     // uncertainty of feature pos

public:
  /// \brief Constructs pose estimator in world frame.
  /// \param K Camera calibration matrix.
  /// \param pixel_std_dev Measurement noise.
  MotionEstimator(const Eigen::Matrix3d& K,
                  const Eigen::Affine3d T_wb,
                  const double pixel_std_dev = 0.5)
  : K_(new gtsam::Cal3_S2(K(0,0), K(1,1), K(0,1), K(0,2), K(1,2)))
  , K_stereo_(new gtsam::Cal3_S2Stereo(K(0,0), K(1,1), K(0,1), K(0,2), K(1,2), 0.537))
  , T_wb0_()
  , feature_noise_( gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345),
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(pixel_std_dev, pixel_std_dev))
      )
    )
  {};

  /// \brief Constructs pose estimator using relative motion description.
  MotionEstimator(const Eigen::Matrix3d& K,
                  const double pixel_std_dev = 0)
  : MotionEstimator(K, Eigen::Affine3d::Identity(), pixel_std_dev)
  {};

  /// \brief Estimates camera pose from 3D-2D correspondences.
  /// \param T_r Initial estimate of relative transformation.
  /// \param image_points 2D image points.
  /// \param world_points 3D planar world points.
  /// \return The results.
  Eigen::Affine3d estimate3D2D(const Eigen::Affine3d T_r,                           
                               const std::vector<cv::Point3f> world_inlier_points,
                               const std::vector<cv::Point2f> image_inlier_points);

};




Eigen::Affine3d MotionEstimator::estimate3D2D(const Eigen::Affine3d T_r,                           /// Camera pose in the world.                                        
                                              const std::vector<cv::Point3f> world_inlier_points,  /// 3D inlier world points in previous frame
                                              const std::vector<cv::Point2f> image_inlier_points)  /// 2D inlier image points in current frame
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

    graph.emplace_shared<ResectioningFactor>(feature_noise_, gtsam::Symbol('x',1), K_,
                                             gtsam::Point2(img_pt.x, img_pt.y),
                                             gtsam::Point3(wrld_pt.x, wrld_pt.y, wrld_pt.z));
  }

  // Set initial estimate.
  initial.insert( gtsam::Symbol('x',1), gtsam::Pose3(T_r.matrix()) );

  // Find the optimal camera pose given correspondences and assumed noise model.
  try
  {
    result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  }
  catch (gtsam::CheiralityException& e)
  {
    return Eigen::Affine3d::Identity();
  }

  // Update pose estimate.
  return Eigen::Affine3d{result.at<gtsam::Pose3>(gtsam::Symbol('x',1)).matrix()};
}



} // namespace BA