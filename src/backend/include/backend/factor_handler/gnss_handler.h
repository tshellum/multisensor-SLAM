#pragma once

/*** ROS packages ***/
#include <ros/ros.h> 
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/NavSatFix.h>
#include "backend/NorthEastHeading.h"

/*** Eigen packages ***/
#include <Eigen/Dense>

/*** GTSAM packages ***/
#include <gtsam/slam/PriorFactor.h> 
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 

/*** Boost packages ***/
#include <boost/property_tree/ptree.hpp>

// Local 
#include "backend/factor_handler/factor_handler.h"



enum { OFFLINE = 0, ONLINE = 1 };


namespace backend
{

namespace factor_handler
{

class GNSSHandler : public FactorHandler<const tf2_msgs::TFMessage&> 
{
private:
  gtsam::noiseModel::Diagonal::shared_ptr noise_; 
  int pose_id_;

public:
  GNSSHandler(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size, 
    std::shared_ptr<Backend> backend,
    boost::property_tree::ptree parameters = boost::property_tree::ptree()
  ) 
  : FactorHandler(nh, topic, queue_size, backend)
  , pose_id_(backend_->getPoseID())
  {
    backend->updateNavStatus(ONLINE);

    if ( parameters != boost::property_tree::ptree() )
    {
      noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        ( gtsam::Vector(6) << gtsam::Vector3::Constant(parameters.get< double >("gnss.orientation_sigma")), 
                              gtsam::Vector3::Constant(parameters.get< double >("gnss.position_sigma"))
        ).finished()
      );
    }
    else
    {
      noise_ = gtsam::noiseModel::Diagonal::Sigmas( 
        ( gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), 
                              gtsam::Vector3::Constant(0.15)
        ).finished()
      );
    }

  }
  ~GNSSHandler() = default; 

  void callback(const tf2_msgs::TFMessage& msg)
  {
    Eigen::Isometry3d T_w = tf2::transformToEigen(msg.transforms[0].transform); 
    gtsam::Pose3 pose(T_w.matrix()); 

    // gtsam::Pose3 delta(gtsam::Rot3::Rodrigues(0.1, 0.1, 0.0), gtsam::Point3(1.0, 1.0, 0.0));


    if (backend_->checkInitialized() == false) 
    {
      pose_id_ = backend_->getPoseID();  
      backend_->setInitialized(true);     
    }
    else
    {
      std::pair<int, bool> associated_id = backend_->searchAssociatedPose(msg.transforms[0].header.stamp);
      pose_id_ = associated_id.first;      
    }

    // ROS_INFO_STREAM("GNSS - id: " << pose_id_);

    gtsam::Key pose_key = gtsam::symbol_shorthand::X(pose_id_);       
    // pose = pose.compose(delta);
    backend_->tryInsertValue(pose_key, pose);
    backend_->addFactor(
      gtsam::PriorFactor<gtsam::Pose3>(
        pose_key, pose, noise_
      )
    );

    backend_->isUpdated();
  }

};


} // namespace factor_handler

} // namespace backend
