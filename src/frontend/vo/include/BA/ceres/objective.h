#pragma once

/*** OpenCV packages ***/
#include "opencv2/calib3d.hpp"

/*** Eigen packages ***/
#include <Eigen/Geometry> 
#include <Eigen/Dense>

/*** Ceres packages ***/
#include "ceres/ceres.h"
#include "ceres/rotation.h"




// The intrinsics need to get combined into a single parameter block;
// use these enums to index instead of numeric constants.
enum 
{
    OFFSET_FOCAL_LENGTH = 0,
    OFFSET_PRINCIPAL_POINT_X,
    OFFSET_PRINCIPAL_POINT_Y,
    OFFSET_K1,
    OFFSET_K2,
    OFFSET_K3,
    OFFSET_P1,
    OFFSET_P2,
};


template <typename T>
inline void apply_radio_distortion_camera_intrinsics(const T &focal_length_x,
                                                     const T &focal_length_y,
                                                     const T &principal_point_x,
                                                     const T &principal_point_y,
                                                     const T &k1,
                                                     const T &k2,
                                                     const T &k3,
                                                     const T &p1,
                                                     const T &p2,
                                                     const T &normalized_x,
                                                     const T &normalized_y,
                                                     T *image_x,
                                                     T *image_y)
{
    T x = normalized_x;
    T y = normalized_y;
    
    // apply distortion to the normalized points to get (xd, yd)
    T r2 = x*x + y*y;
    T r4 = r2 * r2;
    T r6 = r4 * r2;
    T r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    T xd = x * r_coeff + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    T yd = y * r_coeff + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);
    
    // apply focal length and principal point to get the final image coordinates
    *image_x = focal_length_x * xd + principal_point_x;
    *image_y = focal_length_y * yd + principal_point_y;
}





// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct ReprojectionError {
  double imgpt_x;
  double imgpt_y;
  double wrldpt_x;
  double wrldpt_y;
  double wrldpt_z;
  double* const K;
  double* const p;
  double* const P;
  

  // Static parameters
  ReprojectionError(double imgpt_x, double imgpt_y,
                    double wrldpt_x, double wrldpt_y, double wrldpt_z,
                    double f, double c_x, double c_y,
                    double k1, double k2, double k3, double p1, double p2)
  : imgpt_x(imgpt_x)
  , imgpt_y(imgpt_y) 
  , wrldpt_x(wrldpt_x) 
  , wrldpt_y(wrldpt_y) 
  , wrldpt_z(wrldpt_z) 
  , K(new double[8])
  , p(new double[2])
  , P(new double[3])
  {
    K[OFFSET_FOCAL_LENGTH] = f;
    K[OFFSET_PRINCIPAL_POINT_X] = c_x;
    K[OFFSET_PRINCIPAL_POINT_Y] = c_y;
    K[OFFSET_K1] = k1;
    K[OFFSET_K2] = k2;
    K[OFFSET_K3] = k3;
    K[OFFSET_P1] = p1;
    K[OFFSET_P2] = p2;

    p[0] = imgpt_x;
    p[1] = imgpt_y;

    P[0] = wrldpt_x;
    P[1] = wrldpt_y;
    P[2] = wrldpt_z;
  }

  ~ReprojectionError()
  {
    delete [] K;
  }

  // Optimized parameters is inputed
  template <typename T>
  bool operator()(const T* const pose,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    // T p[3];
    // ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    P[0] += pose[3];
    P[1] += pose[4];
    P[2] += pose[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    // compute normalized coordinates
    T xn = P[0] / P[2];
    T yn = P[1] / P[2];
    
    double predicted_x, predicted_y;
    
    // apply distortion to the normalized points to get (xd, yd)
    // do something for zero distortion
    apply_radio_distortion_camera_intrinsics(K[OFFSET_FOCAL_LENGTH],
                                             K[OFFSET_FOCAL_LENGTH],
                                             K[OFFSET_PRINCIPAL_POINT_X],
                                             K[OFFSET_PRINCIPAL_POINT_Y],
                                             K[OFFSET_K1], K[OFFSET_K2], K[OFFSET_K3],
                                            K[OFFSET_P1], K[OFFSET_P2],
                                            xn, yn,
                                            &predicted_x,
                                            &predicted_y);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - imgpt_x; // p[0];
    residuals[1] = predicted_y - imgpt_y; // p[1];

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double imgpt_x, const double imgpt_y,
                                     const double wrldpt_x, const double wrldpt_y, const double wrldpt_z,
                                     const double f, const double c_x, const double c_y,
                                     const double k1, const double k2, const double k3, const double p1, const double p2) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(
        new ReprojectionError(imgpt_x, imgpt_y,
                              wrldpt_x, wrldpt_y, wrldpt_z,
                              f, c_x, c_y,
                              k1, k2, k3, p1, p2)));
  }
};