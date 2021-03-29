#pragma once

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/ProjectionFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use a
// trust-region method known as Powell's Degleg
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

#include <gtsam/slam/PriorFactor.h> 



namespace BA
{


class StructureEstimator
{
private:
    const gtsam::Cal3_S2::shared_ptr K_l_; // Calibration matrix of left camera
    const gtsam::Cal3_S2::shared_ptr K_r_; // Calibration matrix of right camera

    gtsam::Pose3 T_l_; // Pose of left camera
    gtsam::Pose3 T_r_; // Pose of right camera

    const gtsam::SharedNoiseModel feature_noise_; // uncertainty of feature pos
    const gtsam::SharedNoiseModel pose_noise_;    // Set static - Pose should not change

public:
    StructureEstimator(){}
    
    // Stereo using world pose
    StructureEstimator(const Eigen::Matrix3d K_cl,    // Camera matrix for left camera
                       const Eigen::Matrix3d K_cr,    // If stereo: Camera matrix for right camera
                       const Eigen::Affine3d T_wb,    // If world coordinates: World transformation to left camera
                       const double pixel_std_dev)    // Assumed pixel noise
    : K_l_( new gtsam::Cal3_S2(K_cl(0,0), K_cl(1,1), K_cl(0,1), K_cl(0,2), K_cl(1,2)) )
    , K_r_( new gtsam::Cal3_S2(K_cr(0,0), K_cr(1,1), K_cr(0,1), K_cr(0,2), K_cr(1,2)) )
    , T_l_( gtsam::Pose3(T_wb.matrix()) )
    , feature_noise_( gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(pixel_std_dev, pixel_std_dev))
        )
      )
    , pose_noise_( gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0), gtsam::Vector3::Constant(0.0)).finished())
      )
    {};

    // Stereo using relative pose
    StructureEstimator(const Eigen::Matrix3d K_cl,    // Camera matrix for left camera
                       const Eigen::Matrix3d K_cr,    // If stereo: Camera matrix for right camera
                       const double pixel_std_dev)    // Assumed pixel noise
    : StructureEstimator(K_cl, K_cr, Eigen::Affine3d::Identity(), pixel_std_dev)
    {};

    // Mono using world pose
    StructureEstimator(const Eigen::Matrix3d K_cl,    // Camera matrix for left camera
                       const Eigen::Affine3d T_wb,    // If world coordinates: World transformation to left camera/body
                       const double pixel_std_dev)    // Assumed pixel noise
    : StructureEstimator(K_cl, K_cl, T_wb, pixel_std_dev)
    {};
    
    // Mono using relative pose
    StructureEstimator(const Eigen::Matrix3d K_cl,    // Camera matrix for left camera
                       const double pixel_std_dev)    // Assumed pixel noise
    : StructureEstimator(K_cl, K_cl, Eigen::Affine3d::Identity(), pixel_std_dev)
    {};
    

    ~StructureEstimator() {};

    std::vector<cv::Point3f> estimate(const Eigen::Affine3d T_clcr,                             /// Relative transformation from left to right camera
                                      const std::vector<cv::Point3f> world_inlier_points,       /// 3D inlier world points
                                      const std::vector<cv::Point2f> image_inlier_left,         /// 2D inlier image points
                                      const std::vector<cv::Point2f> image_inlier_right = {});  /// 2D inlier image points
};


std::vector<cv::Point3f> StructureEstimator::estimate(const Eigen::Affine3d T_clcr,                       /// Relative transformation from left to right camera
                                                      const std::vector<cv::Point3f> world_inlier_points, /// 3D inlier world points
                                                      const std::vector<cv::Point2f> image_inlier_left,   /// 2D inlier image points
                                                      const std::vector<cv::Point2f> image_inlier_right)  /// 2D inlier image points
{
  if (world_inlier_points.empty())
    return world_inlier_points;

  // Create factor graph.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  gtsam::Values result;

  T_r_ = T_l_.compose( gtsam::Pose3( T_clcr.matrix()) );

  // Insert camera static camera position
  initial.insert<gtsam::Pose3>(gtsam::Symbol('x', 0), T_l_);  
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(gtsam::Symbol('x', 0), T_l_, pose_noise_);

  initial.insert<gtsam::Pose3>(gtsam::Symbol('x', 1), T_r_);  
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(gtsam::Symbol('x', 1), T_r_, pose_noise_);


  // Insert features using GenericProjectionFactor and an initial estimate of the 3D location of the landmark
  for (size_t i = 0; i < world_inlier_points.size(); i++) 
  {
    // Left image
    gtsam::Point2 img_pt_l = gtsam::Point2(image_inlier_left[i].x, 
                                           image_inlier_left[i].y);

    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3>>(
        img_pt_l, feature_noise_, gtsam::Symbol('x', 0), gtsam::Symbol('l', i), K_l_);


    // Right image
    gtsam::Point2 img_pt_r = gtsam::Point2(image_inlier_right[i].x, 
                                           image_inlier_right[i].y);

    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3> >(
        img_pt_r, feature_noise_, gtsam::Symbol('x', 1), gtsam::Symbol('l', i), K_r_);
  

    // Initial estimate of "world" point position
    gtsam::Point3 wrld_pt = gtsam::Point3(world_inlier_points[i].x, 
                                          world_inlier_points[i].y, 
                                          world_inlier_points[i].z);

    initial.insert<gtsam::Point3>(gtsam::Symbol('l', i), wrld_pt);  
  }


  // Optimize  
  try
  {
    result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  }
  catch (gtsam::CheiralityException& e)
  {
    return {};
  }


  // Update world points
  std::vector<cv::Point3f> wrld_pts_optim;
  for (size_t i = 0; i < world_inlier_points.size(); i++) 
  {
    Eigen::Vector3d wrld_pt_opt = result.at<gtsam::Point3>(gtsam::Symbol('l', i)).matrix();
    wrld_pts_optim.push_back( cv::Point3f(wrld_pt_opt.x(), wrld_pt_opt.y(), wrld_pt_opt.z()) );

    ROS_INFO_STREAM( "Original[" << i << "]: " << world_inlier_points[i] );
    ROS_INFO_STREAM( "Optimized[" << i << "]: " << wrld_pts_optim[i] );
    ROS_INFO_STREAM( "Diff[" << i << "]: " << world_inlier_points[i] - wrld_pts_optim[i] );
  }

  return wrld_pts_optim;
}


} // namespace BA