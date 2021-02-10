#include "gtsam_pose_estimator.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"

#include "gtsam/geometry/SimpleCamera.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"

// Heavily inspired by the example CameraResectioning.cpp in GTSAM with copyright:
//
// Copyright (c) 2010, Georgia Tech Research Corporation
// Atlanta, Georgia 30332-0415
// All Rights Reserved

using gtsam::symbol_shorthand::X;

/**
 * Unary factor on the unknown pose, resulting from measuring the projection of
 * a known 3D point in the image
 */
class ResectioningFactor: public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  typedef NoiseModelFactor1<gtsam::Pose3> Base;

  gtsam::Cal3_S2::shared_ptr K_; ///< camera's intrinsic parameters
  gtsam::Point3 P_;              ///< 3D point on the calibration rig
  gtsam::Point2 p_;              ///< 2D measurement of the 3D point

public:

  /// Construct factor given known point P and its projection p
  ResectioningFactor(const gtsam::SharedNoiseModel& model,
                     const gtsam::Key& key,
                     const gtsam::Cal3_S2::shared_ptr& calib,
                     const gtsam::Point2& p,
                     const gtsam::Point3& P)
      : Base(model, key)
      , K_(calib)
      , P_(P)
      , p_(p)
  {}

  /// evaluate the error
  virtual gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                                      boost::optional<gtsam::Matrix&> H =  boost::none) const
  {
    gtsam::SimpleCamera camera(pose, *K_);
    return camera.project(P_, H, boost::none, boost::none) - p_;
  }
};


GtsamPoseEstimator::GtsamPoseEstimator(PoseEstimator::Ptr initial_pose_estimator, const Eigen::Matrix3d& K)
    : initial_pose_estimator_{initial_pose_estimator}
    , K_{K}
    , K_gtsam_(new gtsam::Cal3_S2(K_(0,0), K_(1,1), K_(0,1), K_(0,2), K_(1,2)))
{ }

PoseEstimate GtsamPoseEstimator::estimate(const std::vector<cv::Point2f>& image_points,
                                          const std::vector<cv::Point3f>& world_points)
{
  // Get initial pose estimate.
  PoseEstimate init_estimate = initial_pose_estimator_->estimate(image_points, world_points);

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
  const auto& init_pose = init_estimate.pose_W_C;
  gtsam::Values initial;
  initial.insert(X(1), gtsam::Pose3(gtsam::Rot3(init_pose.rotationMatrix()), init_pose.translation()));

  // Find the optimal camera pose given correspondences and assumed noise model.
  gtsam::Values result;
  try
  {
    result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  }
  catch (gtsam::CheiralityException& e)
  {
    return {};
  }

  // Update pose estimate.
  init_estimate.pose_W_C = Sophus::SE3d{result.at<gtsam::Pose3>(X(1)).matrix()};
  return init_estimate;
}
