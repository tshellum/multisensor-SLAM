#pragma once

#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"

#include "gtsam/geometry/SimpleCamera.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include <gtsam/slam/SmartProjectionPoseFactor.h>

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

  gtsam::Pose3 T_wb0_;

  // Use same robust measurement noise model.
  const gtsam::SharedNoiseModel feature_noise_;     // uncertainty of feature pos
  const gtsam::SharedNoiseModel landmark_noise_;     // uncertainty of feature pos
  const gtsam::SharedNoiseModel pose_noise_prior_;  // Set static - Pose should not change
  const gtsam::SharedNoiseModel pose_noise_;        

public:
  /// \brief Constructs pose estimator in world frame.
  /// \param K Camera calibration matrix.
  /// \param pixel_std_dev Measurement noise.
  MotionEstimator(const Eigen::Matrix3d& K,
                  const Eigen::Affine3d T_wb,
                  const double pixel_std_dev = 0.5,
                  const double rot_std_dev   = M_PI/18, // 10 degrees standard dev
                  const double trans_std_dev = 3) 
  : K_(new gtsam::Cal3_S2(K(0,0), K(1,1), K(0,1), K(0,2), K(1,2)))
  , T_wb0_()
  , feature_noise_( gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345),
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(pixel_std_dev, pixel_std_dev))
      )
    )
  , landmark_noise_( gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345),
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(0.0))
      )
    )
  , pose_noise_prior_( gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0), gtsam::Vector3::Constant(0.0)).finished())
    )
  , pose_noise_( gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345),
      gtsam::noiseModel::Diagonal::Sigmas(
        ( gtsam::Vector(6) << gtsam::Vector3::Constant(rot_std_dev), gtsam::Vector3(0.5, 0.5, trans_std_dev)).finished()
        )
      )
    )
  {};

  /// \brief Constructs pose estimator using relative motion description.
  MotionEstimator(const Eigen::Matrix3d& K,
                  const double pixel_std_dev = 0,
                  const double rot_std_dev   = M_PI/18,
                  const double trans_std_dev = 3)
  : MotionEstimator(K, Eigen::Affine3d::Identity(), pixel_std_dev, rot_std_dev, trans_std_dev)
  {};


  /// \brief Estimates camera pose from 3D-2D correspondences.
  /// \param T_r Initial estimate of relative transformation.
  /// \param image_points 2D image points.
  /// \param world_points 3D planar world points.
  /// \return The results.
  Eigen::Affine3d estimate3D2D(const Eigen::Affine3d T_clcr,                           
                               const std::vector<cv::Point3f> world_inlier_points,
                               const std::vector<cv::Point2f> image_inlier_points);

  /// \brief Estimates camera pose from 3D-2D correspondences.
  /// \param T_r Initial estimate of relative transformation.
  /// \param image_points 2D image points.
  /// \param world_points 3D planar world points.
  /// \return The results.
  Eigen::Affine3d estimate3D2Dresectioning(const Eigen::Affine3d T_r,                           
                                           const std::vector<cv::Point3f> world_inlier_points,
                                           const std::vector<cv::Point2f> image_inlier_points);

  /// \brief Estimates camera pose from 3D-3D correspondences.
  /// \param T_r Initial estimate of relative transformation.
  /// \param image_points 2D image points.
  /// \param world_points 3D planar world points.
  /// \return The results.
  Eigen::Affine3d estimate3D3D(const Eigen::Affine3d T_clcr,                           
                               const std::vector<cv::Point3f> world_points_left,
                               const std::vector<cv::Point3f> world_points_right);
};




Eigen::Affine3d MotionEstimator::estimate3D2D(const Eigen::Affine3d T_clcr,                        /// Camera pose in the world.                                        
                                              const std::vector<cv::Point3f> world_inlier_points,  /// 3D inlier world points 
                                              const std::vector<cv::Point2f> image_inlier_points)  /// 2D inlier image points
{
  if (world_inlier_points.empty())
    return T_clcr;

  // Create factor graph.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  gtsam::Values result;

  gtsam::Pose3 T_wb = T_wb0_.compose( gtsam::Pose3( T_clcr.matrix()) );

  // Insert camera position
  initial.insert<gtsam::Pose3>(gtsam::Symbol('x', 0), T_wb0_);  
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(gtsam::Symbol('x', 0), T_wb0_, pose_noise_prior_);

  initial.insert<gtsam::Pose3>(gtsam::Symbol('x', 1), T_wb);  
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(gtsam::Symbol('x', 1), T_wb, pose_noise_);


  // Insert features using GenericProjectionFactor and an initial 
  for (size_t i = 0; i < world_inlier_points.size(); i++) 
  {
    // Left image
    gtsam::Point2 img_pt = gtsam::Point2(image_inlier_points[i].x, 
                                         image_inlier_points[i].y);

    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
        img_pt, feature_noise_, gtsam::Symbol('x', 1), gtsam::Symbol('l', i), K_);

    
    // Initial estimate of "world" point position
    gtsam::Point3 wrld_pt = gtsam::Point3(world_inlier_points[i].x, 
                                          world_inlier_points[i].y, 
                                          world_inlier_points[i].z);

    initial.insert<gtsam::Point3>(gtsam::Symbol('l', i), wrld_pt);  
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(gtsam::Symbol('l', i), wrld_pt, landmark_noise_);
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
  return Eigen::Affine3d{result.at<gtsam::Pose3>(gtsam::symbol('x', 1)).matrix()};
}


// Eigen::Affine3d MotionEstimator::estimate3D2D(const Eigen::Affine3d T_clcr,                        /// Camera pose in the world.                                        
//                                               const std::vector<cv::Point3f> world_inlier_points,  /// 3D inlier world points 
//                                               const std::vector<cv::Point2f> image_inlier_points)  /// 2D inlier image points
// {
//   if (world_inlier_points.empty())
//     return T_clcr;

//   // Create factor graph.
//   gtsam::NonlinearFactorGraph graph;
//   gtsam::Values initial;
//   gtsam::Values result;

//   gtsam::Pose3 T_wb = T_wb0_.compose( gtsam::Pose3( T_clcr.matrix()) );

//   // Insert camera position
//   initial.insert<gtsam::Pose3>(gtsam::Symbol('x', 0), T_wb0_);  
//   graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(gtsam::Symbol('x', 0), T_wb0_, pose_noise_prior_);

//   initial.insert<gtsam::Pose3>(gtsam::Symbol('x', 1), T_wb);  
//   graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(gtsam::Symbol('x', 1), T_wb, pose_noise_);


//   // Insert features using GenericProjectionFactor and an initial 
//   for (size_t i = 0; i < world_inlier_points.size(); i++) 
//   {
//     // Left image
//     gtsam::Point2 img_pt = gtsam::Point2(image_inlier_points[i].x, 
//                                          image_inlier_points[i].y);

//     graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
//         img_pt, feature_noise_, gtsam::Symbol('x', 1), gtsam::Symbol('l', i), K_);

    
//     // Initial estimate of "world" point position
//     gtsam::Point3 wrld_pt = gtsam::Point3(world_inlier_points[i].x, 
//                                           world_inlier_points[i].y, 
//                                           world_inlier_points[i].z);

//     initial.insert<gtsam::Point3>(gtsam::Symbol('l', i), wrld_pt);  
//     graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(gtsam::Symbol('l', i), wrld_pt, landmark_noise_);
//   }


//   // Find the optimal camera pose given correspondences and assumed noise model.
//   try
//   {
//     result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
//   }
//   catch (gtsam::CheiralityException& e)
//   {
//     return Eigen::Affine3d::Identity();
//   }

//   // Update pose estimate.
//   return Eigen::Affine3d{result.at<gtsam::Pose3>(gtsam::symbol('x', 1)).matrix()};
// }






Eigen::Affine3d MotionEstimator::estimate3D2Dresectioning(const Eigen::Affine3d T_r,                           /// Camera pose in the world.                                        
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





// Eigen::Affine3d MotionEstimator::estimate3D3D(const Eigen::Affine3d T_clcr,                       /// Relative motion in body.                                        
//                                               const std::vector<cv::Point3f> world_points_left,   /// 3D inlier world points 
//                                               const std::vector<cv::Point3f> world_points_right)  /// 2D inlier image points
// {
//   if (world_points_left.empty())
//     return T_clcr;

//   // Create factor graph.
//   gtsam::NonlinearFactorGraph graph;
//   gtsam::Values initial;
//   gtsam::Values result;

//   gtsam::Pose3 T_r = gtsam::Pose3( T_clcr.matrix());

//   // Insert camera position
//   initial.insert<gtsam::Pose3>(gtsam::Symbol('x', 0), T_r);  

//   // Insert features using GenericProjectionFactor and an initial 
//   for (size_t i = 0; i < world_points_left.size(); i++) 
//   {
//     const cv::Point3f wpt_l  = world_points_left[i];
//     const cv::Point3f wpt_r = world_points_right[i];

//     graph.emplace_shared<ScaledPoseFactor>(landmark_noise_, gtsam::Symbol('x', 0), K_,
//                                            gtsam::Point3(wpt_l.x, wpt_l.y, wpt_l.z),
//                                            gtsam::Point3(wpt_r.x, wpt_r.y, wpt_r.z));
//   }

//   // Find the optimal camera pose given correspondences and assumed noise model.
//   try
//   {
//     result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
//   }
//   catch (gtsam::CheiralityException& e)
//   {
//     return Eigen::Affine3d::Identity();
//   }

//   // Update pose estimate.
//   return Eigen::Affine3d{result.at<gtsam::Pose3>(gtsam::symbol('x', 1)).matrix()};
// }



} // namespace BA