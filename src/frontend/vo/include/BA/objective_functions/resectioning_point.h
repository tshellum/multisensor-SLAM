#pragma once

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


/**
 * Unary factor on the unknown pose, resulting from measuring the projection of
 * a known 3D point in the image
 */
class ResectioningPointFactor: public gtsam::NoiseModelFactor1<gtsam::Point3>
{
  typedef NoiseModelFactor1<gtsam::Point3> Base; // Dette bestemmer optimalisert variabel??

  gtsam::Cal3_S2::shared_ptr K_; ///< camera's intrinsic parameters
  // gtsam::Point3 P_;              ///< 3D point on the calibration rig
  gtsam::Point2 p_;              ///< 2D measurement of the 3D point
  gtsam::Pose3 T_;
  gtsam::PinholeCamera<gtsam::Cal3_S2> camera_;

public:

  /// Construct factor given known point P and its projection p
  ResectioningPointFactor(const gtsam::SharedNoiseModel& model,
                          const gtsam::Key& key,
                          const gtsam::Cal3_S2::shared_ptr& calib,
                          const gtsam::Point2& p,
                          const gtsam::Pose3& T)
      : Base(model, key)
      , K_(calib)
      , p_(p)
      , T_(T)
      , camera_(T_, *K_)
  {}

  /// evaluate the error
  virtual gtsam::Vector evaluateError(const gtsam::Point3& P,
                                      boost::optional<gtsam::Matrix&> H =  boost::none) const
  {
    return camera_.project(P, H, boost::none, boost::none) - p_;
  }
};
