#pragma once

#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include "gtsam/geometry/SimpleCamera.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include <gtsam/slam/BetweenFactor.h>

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

  gtsam::Pose3 T_stereo_;

  // Use same robust measurement noise model.
  const gtsam::SharedNoiseModel feature_noise_;  // uncertainty of feature pos
  const gtsam::SharedNoiseModel pose_noise_;     // Set static - Pose should not change

public:
  /// \brief Constructs pose estimator in world frame.
  /// \param K Camera calibration matrix.
  /// \param pixel_std_dev Measurement noise.
  MotionEstimator(const Eigen::Matrix3d& K,
                  const Eigen::Affine3d T_stereo,
                  const double pixel_std_dev = 0.5)
  : K_( new gtsam::Cal3_S2(K(0,0), K(1,1), K(0,1), K(0,2), K(1,2)) )
  , K_stereo_( new gtsam::Cal3_S2Stereo(K(0,0), K(1,1), K(0,1), K(0,2), K(1,2), 0.537) )
  , T_stereo_( gtsam::Pose3(T_stereo.matrix()) )
  , feature_noise_( gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345),
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(pixel_std_dev, pixel_std_dev))
      )
    )
  , pose_noise_( gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0), gtsam::Vector3::Constant(0.0)).finished())
    )

  {};

  /// \brief Constructs pose estimator using relative motion description.
  MotionEstimator(const Eigen::Matrix3d& K,
                  const double pixel_std_dev = 0.5)
  : MotionEstimator(K, Eigen::Affine3d::Identity(), pixel_std_dev)
  {};

  // bool isValidStereo() {return T_stereo_ == gtsam::Pose3::identity(); }
  bool isValidStereo() {return !(T_stereo_.equals( gtsam::Pose3::identity()) ); }

  /// \brief Estimates camera pose from 3D-2D correspondences.
  /// \param T_r Initial estimate of relative transformation.
  /// \param world_points 3D planar world points.
  /// \param image_points 2D image points for left and right camera.
  /// \return The results.
  Eigen::Affine3d estimate3D2D(const Eigen::Affine3d T_r,      
                               const std::vector<cv::Point3f> world_points,
                               const std::vector<cv::Point2f> image_points_left,
                               const std::vector<cv::Point2f> image_points_right = {});

};




Eigen::Affine3d MotionEstimator::estimate3D2D(const Eigen::Affine3d T_r,     
                                              const std::vector<cv::Point3f> world_points,
                                              const std::vector<cv::Point2f> image_points_left,
                                              const std::vector<cv::Point2f> image_points_right) 
{
  // Create factor graph.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  gtsam::Values result;

  // Add each 3D-2D correspondence to the factor graph.
  for (size_t i=0; i<world_points.size(); ++i)
  {
    const cv::Point2f img_pt_l  = image_points_left[i];
    const cv::Point3f wrld_pt = world_points[i];

    graph.emplace_shared<ResectioningFactor>(feature_noise_, gtsam::Symbol('x',1), K_,
                                             gtsam::Point2(img_pt_l.x, img_pt_l.y),
                                             gtsam::Point3(wrld_pt.x, wrld_pt.y, wrld_pt.z));

    if ( isValidStereo() )
    {
      const cv::Point2f img_pt_r  = image_points_right[i];
      graph.emplace_shared<ResectioningFactor>(feature_noise_, gtsam::Symbol('x',2), K_,
                                              gtsam::Point2(img_pt_r.x, img_pt_r.y),
                                              gtsam::Point3(wrld_pt.x, wrld_pt.y, wrld_pt.z));
    }

  }

  gtsam::Pose3 pose_wb = gtsam::Pose3(T_r.matrix());
  initial.insert( gtsam::Symbol('x',1), pose_wb );

  if ( isValidStereo() )
  {
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('x',1), gtsam::Symbol('x',2), T_stereo_, pose_noise_
      ));

    initial.insert( gtsam::Symbol('x',2), pose_wb * T_stereo_ );
  }
  

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