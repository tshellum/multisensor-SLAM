#pragma once

// #include "gtsam_pose_estimator.h"
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
