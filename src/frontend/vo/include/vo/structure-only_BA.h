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

// #include "gtsam_pose_estimator.h"
// #include "opencv2/calib3d.hpp"
// #include "opencv2/core/eigen.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



class StructureOnlyBA
{
private:
    const gtsam::Cal3_S2::shared_ptr K_l_;
    const gtsam::Cal3_S2::shared_ptr K_r_;

    const gtsam::Pose3 T_l_; 
    const gtsam::Pose3 T_r_; 

    const gtsam::SharedNoiseModel feature_noise_; // uncertainty of feature pos
    const gtsam::SharedNoiseModel pose_noise_;    // Set static

public:
    StructureOnlyBA(const Eigen::Matrix3d K_l,        // Camera matrix for left camera
                    const Eigen::Matrix3d K_r,        // Camera matrix for right camera
                    const Eigen::Affine3d T_r,        // Relative transformation from left to right camera
                    const double pixel_std_dev = 0.5) // Assumed pixel noise
    : K_l_( new gtsam::Cal3_S2(K_l(0,0), K_l(1,1), K_l(0,1), K_l(0,2), K_l(1,2)) ),
      K_r_( new gtsam::Cal3_S2(K_r(0,0), K_r(1,1), K_r(0,1), K_r(0,2), K_r(1,2)) ),
      T_l_( gtsam::Pose3::identity() ),
      T_r_( gtsam::Pose3(T_r.matrix()) ),
      feature_noise_( gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(pixel_std_dev, pixel_std_dev))
        )
      ),
      pose_noise_( gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0), gtsam::Vector3::Constant(0.0)).finished())
      )
    {};
    ~StructureOnlyBA() {};

    void estimate(std::vector<cv::KeyPoint> image_inlier_left,    /// 2D inlier image points
                  std::vector<cv::KeyPoint> image_inlier_right,   /// 2D inlier image points
                  pcl::PointCloud<pcl::PointXYZ>::Ptr world_inlier_points); /// 3D inlier world points);
};


void StructureOnlyBA::estimate(std::vector<cv::KeyPoint> image_inlier_left,    /// 2D inlier image points
                               std::vector<cv::KeyPoint> image_inlier_right,   /// 2D inlier image points
                               pcl::PointCloud<pcl::PointXYZ>::Ptr world_inlier_points)  /// 3D inlier world points
{
  // Create factor graph.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  // Create camera objects
  gtsam::PinholeCamera<gtsam::Cal3_S2> camera_l(T_l_, *K_l_);
  gtsam::PinholeCamera<gtsam::Cal3_S2> camera_r(T_r_, *K_r_);

  graph.addPrior(gtsam::Symbol('x', 0), T_l_, pose_noise_);  // add directly to graph
  graph.addPrior(gtsam::Symbol('x', 1), T_r_, pose_noise_);  // add directly to graph

  for (size_t i = 0; i < world_inlier_points->points.size(); ++i) {
    // gtsam::Point3 pt3D = gtsam::Point3(world_inlier_points[i].x, 
    //                                    world_inlier_points[i].y, 
    //                                    world_inlier_points[i].z);
    gtsam::Point3 pt3D = gtsam::Point3(world_inlier_points->points[i].x, 
                                       world_inlier_points->points[i].y, 
                                       world_inlier_points->points[i].z);

    gtsam::Point2 pt_l = camera_l.project(pt3D);
    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
        pt_l, feature_noise_, gtsam::Symbol('x', 0), gtsam::Symbol('l', i), K_l_);

    gtsam::Point2 pt_r = camera_r.project(pt3D);
    graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
        pt_r, feature_noise_, gtsam::Symbol('x', 1), gtsam::Symbol('l', i), K_r_);  

    initial.insert<gtsam::Point3>(gtsam::Symbol('l', i), pt3D);  
  }

  initial.print("Initial Estimates:\n");

  // Optimize
  gtsam::Values result;
  try
  {
    result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  }
  catch (gtsam::CheiralityException& e)
  {
    // ROS_ERROR(e);
  }

  result.print("Optimized results:\n");
}
