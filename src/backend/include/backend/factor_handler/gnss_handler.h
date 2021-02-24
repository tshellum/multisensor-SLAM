#pragma once

/*** ROS packages ***/
#include <ros/ros.h> 
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/NavSatFix.h>

#include <Eigen/Dense>


/*** GTSAM packages ***/
#include <gtsam/slam/PriorFactor.h> 
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 

// Local 
#include "backend/factor_handler/factor_handler.h"

namespace backend
{

namespace factor_handler
{

class GNSSHandler : public FactorHandler<const tf2_msgs::TFMessage&> 
{
private:
  const gtsam::noiseModel::Diagonal::shared_ptr noise_; 
public:
  GNSSHandler(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size, 
    std::shared_ptr<Backend> backend
  ) : FactorHandler(nh, topic, queue_size, backend), 
      noise_( 
        gtsam::noiseModel::Diagonal::Sigmas( 
          ( gtsam::Vector6() 
            << gtsam::Vector3(0.1, 0.1, 0.1), 
            gtsam::Vector3(0.15, 0.15, 0.1) 
          ).finished() 
        ) 
      )
  {}
  ~GNSSHandler() = default; 

  void callback(const tf2_msgs::TFMessage& msg)
  {
    Eigen::Isometry3d T_w = tf2::transformToEigen(msg.transforms[0].transform); 
    gtsam::Pose3 pose(T_w.matrix()); 

    gtsam::Key poseKey = gtsam::symbol_shorthand::X(backend_->getPoseID()); 
    if (backend_->getValues().exists(poseKey))
      backend_->getValues().insert(poseKey, pose); 
    
    backend_->getGraph().add(
      gtsam::PriorFactor<gtsam::Pose3>(
        poseKey, pose, noise_
      )
    ); 
  }

  void addPoseFactor() 
  {

  }
};
/**
 * 
  if (! initial_estimate.exists(gtsam::symbol_shorthand::X(pose_id)))
    initial_estimate.insert(gtsam::symbol_shorthand::X(pose_id), pose);

  graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id), 
    
 * */

} // namespace factor_handler

} // namespace backend
