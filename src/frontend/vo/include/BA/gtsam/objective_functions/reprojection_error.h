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
 * Unary factor on the unknown landmark, resulting from measuring the projection of
 * a the landmark onto the image with a known pose
 */
class ReprojectionFactor: public gtsam::NoiseModelFactor1<gtsam::Point3>
{
  typedef NoiseModelFactor1<gtsam::Point3> Base; 

  int id_;
  gtsam::Point2 p_;              ///< 2D measurement of the 3D point
  gtsam::PinholeCamera<gtsam::Cal3_S2> camera_; ///< camera's intrinsic and extrinsic parameters

public:

  /// Construct factor given known point P and its projection p
  ReprojectionFactor(const gtsam::SharedNoiseModel& model,
                     const gtsam::Key& key,
                     const gtsam::Cal3_S2::shared_ptr& calib,
                     const gtsam::Point2& p,
                     const gtsam::Pose3& T,
                     const int id)
      : Base(model, key)
      , p_(p)
      , camera_(T, *calib)
      , id_(id)
  {}

  /// evaluate the error
  virtual gtsam::Vector evaluateError(const gtsam::Point3& P,
                                      boost::optional<gtsam::Matrix&> H =  boost::none) const override
  {
    return camera_.project(P, H, boost::none, boost::none) - p_;
  }
};
