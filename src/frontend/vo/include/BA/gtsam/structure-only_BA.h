#pragma once

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use a
// trust-region method known as Powell's Degleg
// #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

#include "objective_functions/reprojection_error.h"


namespace BA
{


class StructureEstimator
{
private:
    const gtsam::Cal3_S2::shared_ptr K_l_;  // Calibration matrix of left camera
    const gtsam::Cal3_S2::shared_ptr K_r_;  // Calibration matrix of right camera

    gtsam::Pose3 T_stereo_; // Relative pose

    const gtsam::SharedNoiseModel feature_noise_; // uncertainty of feature pos
    const gtsam::SharedNoiseModel pose_noise_;    // Set static - Pose should not change
    const gtsam::SharedNoiseModel point_noise_;   // Set static - Pose should not change

public:
    StructureEstimator(){}
    
    // Stereo with different calibraiton
    StructureEstimator(const Eigen::Matrix3d K_cl,        // Camera matrix for left camera
                       const Eigen::Matrix3d K_cr,        // If stereo: Camera matrix for right camera
                       const Eigen::Affine3d T_stereo,    // If world coordinates: World transformation to left camera
                       const double pixel_std_dev = 0.5)  // Assumed pixel noise
    : K_l_( new gtsam::Cal3_S2(K_cl(0,0), K_cl(1,1), K_cl(0,1), K_cl(0,2), K_cl(1,2)) )
    , K_r_( new gtsam::Cal3_S2(K_cr(0,0), K_cr(1,1), K_cr(0,1), K_cr(0,2), K_cr(1,2)) )
    , T_stereo_( gtsam::Pose3(T_stereo.matrix()) )
    , feature_noise_( gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(1.345),
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(pixel_std_dev, pixel_std_dev)
          )
        )
      )
    , pose_noise_( gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0), gtsam::Vector3::Constant(0.0)).finished()
        )
      )
    , point_noise_( gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(1.345),
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1.0, 1.0, 2.0)
          )
        )
      )
    {};

    // Stereo with same calibraiton
    StructureEstimator(const Eigen::Matrix3d K_cl,        // Camera matrix for left camera
                       const Eigen::Affine3d T_stereo,    // If world coordinates: World transformation to left camera/body
                       const double pixel_std_dev = 0.5)  // Assumed pixel noise
    : StructureEstimator(K_cl, K_cl, T_stereo, pixel_std_dev)
    {};

    ~StructureEstimator() {};

    std::vector<cv::Point3f> estimate(const std::vector<cv::Point3f> world_inlier_points,  /// 3D inlier world points
                                      const std::vector<cv::Point2f> image_inlier_left,    /// 2D inlier image points
                                      const std::vector<cv::Point2f> image_inlier_right);  /// 2D inlier image points
};


std::vector<cv::Point3f> StructureEstimator::estimate(const std::vector<cv::Point3f> world_inlier_points, /// 3D inlier world points
                                                      const std::vector<cv::Point2f> image_inlier_left,   /// 2D inlier image points
                                                      const std::vector<cv::Point2f> image_inlier_right)  /// 2D inlier image points
{
  if (world_inlier_points.empty())
    return world_inlier_points;

  // Create factor graph.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  gtsam::Values result;


  // Insert features using ReprojectionFactor and an initial estimate of the 3D location of the landmark
  for (size_t i = 0; i < world_inlier_points.size(); i++) 
  {
    // Left image
    gtsam::Point2 img_pt_l = gtsam::Point2(image_inlier_left[i].x, 
                                           image_inlier_left[i].y);

    graph.emplace_shared<ReprojectionFactor>(feature_noise_, 
                                             gtsam::Symbol('l', i), 
                                             K_l_,
                                             img_pt_l,
                                             gtsam::Pose3::identity(),
                                             i);

    // Right image
    gtsam::Point2 img_pt_r = gtsam::Point2(image_inlier_right[i].x, 
                                           image_inlier_right[i].y);

    graph.emplace_shared<ReprojectionFactor>(feature_noise_, 
                                             gtsam::Symbol('l', i), 
                                             K_r_,
                                             img_pt_r,
                                             T_stereo_,
                                             i);


    // Initial estimate of "world" point position
    gtsam::Point3 wrld_pt = gtsam::Point3(world_inlier_points[i].x, 
                                          world_inlier_points[i].y, 
                                          world_inlier_points[i].z);


    initial.insert<gtsam::Point3>(gtsam::Symbol('l', i), wrld_pt); 
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(gtsam::Symbol('l', i), wrld_pt, point_noise_);
  }


  // Optimize  
  try
  {
    result = gtsam::GaussNewtonOptimizer(graph, initial).optimize();
  }
  catch (std::exception& e)
  {
    result = initial;
  }


  // Update world points
  std::vector<cv::Point3f> wrld_pts_optim;
  double x_abs, y_abs, z_abs;
  for (size_t i = 0; i < world_inlier_points.size(); i++) 
  {
    Eigen::Vector3d wrld_pt_opt = result.at<gtsam::Point3>(gtsam::Symbol('l', i)).matrix();
    wrld_pts_optim.push_back( cv::Point3f(wrld_pt_opt.x(), wrld_pt_opt.y(), wrld_pt_opt.z()) );

    cv::Point3f diff = world_inlier_points[i] - wrld_pts_optim[i];
    x_abs += abs(diff.x);
    y_abs += abs(diff.y);
    z_abs += abs(diff.z);
  }
  
  ROS_INFO_STREAM( "MAE - x: " << x_abs / world_inlier_points.size() << ", y: " << y_abs / world_inlier_points.size() << ", z: " << z_abs / world_inlier_points.size() );

  return wrld_pts_optim;
}


} // namespace BA