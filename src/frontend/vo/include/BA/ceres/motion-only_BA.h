#pragma once

/*** OpenCV packages ***/
#include "opencv2/calib3d.hpp"

/*** Eigen packages ***/
#include <Eigen/Geometry> 
#include <Eigen/Dense>

/*** Ceres packages ***/
#include "ceres/ceres.h"
#include "ceres/rotation.h"

/*** Local ***/
// #include "objective.h"
#include "reprojection_error.h"


namespace BA
{


class MotionEstimator
{
private:
  ceres::Solver::Options options_;

public:
  MotionEstimator(int num_iterations = 200)
  {
    options_.max_num_iterations = num_iterations;
    options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  };

  ~MotionEstimator()
  {};

  ceres::Problem generateResiduals(const Eigen::Affine3d T_r,                           
                                   const std::vector<cv::Point3f> world_inlier_points,
                                   const std::vector<cv::Point2f> image_inlier_points);

  bool SolveOptimizationProblem(ceres::Problem* problem);
};


ceres::Problem MotionEstimator::generateResiduals(const Eigen::Affine3d T_r,
                                                  const std::vector<cv::Point3f> world_inlier_points,
                                                  const std::vector<cv::Point2f> image_inlier_points)
{
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < image_inlier_points.size(); ++i) 
  {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function = ReprojectionError::Create(image_inlier_points[i].x, image_inlier_points[i].y);

    Eigen::Vector3d r = T_r.linear().eulerAngles(0,1,2);
    Eigen::Vector3d t = T_r.translation();

    double* intrinsics{ new double[9]{ r.x(), r.y(), r.z(), t.x(), t.y(), t.z(), 718.856, 607.1928, 185.2157 } };
    double* point{ new double[3]{ world_inlier_points[i].x, 
                                  world_inlier_points[i].y, 
                                  world_inlier_points[i].z } };

    ROS_INFO_STREAM("T_r: \n" << T_r.matrix());
    // ROS_INFO_STREAM("r: \n" << r);
    // ROS_INFO_STREAM("t: \n" << t);
    ROS_INFO_STREAM("world_inlier_points: " << world_inlier_points[i]);

    for (int j = 0; j < 9; j++) 
    {
      ROS_INFO_STREAM("intrinsics[" << j << "]: " << intrinsics[j]);
    }

    for (int j = 0; j < 3; j++) 
    {
      ROS_INFO_STREAM("point[" << j << "]: " << point[j]);
    }


    // ceres::CostFunction* cost_function = ReprojectionError::Create(
    //   image_inlier_points[i].x, image_inlier_points[i].y,
    //   world_inlier_points[i].x, world_inlier_points[i].y, world_inlier_points[i].z,
    //   1, 2, 3,
    //   5, 6, 7, 8, 9);


    // Use Huber's loss function.
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

    // problem.AddResidualBlock(cost_function,
    //                          loss_function,
    //                          0.0,
    //                          0.0);
  }

  return problem;
}
// Returns true if the solve was successful.
bool MotionEstimator::SolveOptimizationProblem(ceres::Problem* problem) 
{
  CHECK(problem != NULL);

  ceres::Solver::Summary summary;
  ceres::Solve(options_, problem, &summary);

  ROS_INFO_STREAM( summary.FullReport() << '\n' );

  return summary.IsSolutionUsable();
}


} // namespace BA