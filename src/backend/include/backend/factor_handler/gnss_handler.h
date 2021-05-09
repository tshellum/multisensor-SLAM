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
  ros::Time prev_stamp_;

  bool use_gnss_;

public:
  GNSSHandler(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size, 
    std::shared_ptr<Backend> backend,
    boost::property_tree::ptree parameters = boost::property_tree::ptree()  
  ) 
  : FactorHandler(nh, topic, queue_size, backend, parameters.get< bool >("sensor_status.gnss", false))
  , pose_id_(backend_->getPoseID())
  , noise_( gtsam::noiseModel::Diagonal::Sigmas(
      ( gtsam::Vector(6) << gtsam::Vector3::Constant( parameters.get< double >("gnss.orientation_sigma", M_PI/180) ), 
                            gtsam::Vector3::Constant( parameters.get< double >("gnss.position_sigma", 0.1) )
      ).finished() )
    )
  {
    if (online_)
    {
      backend->updateNavStatus(ONLINE); 
      std::cout << "- GNSS" << std::endl;
    }
  }
  ~GNSSHandler() = default; 


  void callback(const tf2_msgs::TFMessage& msg)
  {
    if (msg.transforms[0].header.stamp < ros::Time(1317639337, 634870052) )
      return;

    Eigen::Isometry3d T_w = tf2::transformToEigen(msg.transforms[0].transform); 
    gtsam::Pose3 pose(T_w.matrix()); 

    gtsam::Pose3 delta;
    bool assocated = true;
    if (backend_->checkInitialized() == false) 
    {
      pose_id_ = backend_->getPoseID();  
      backend_->setInitialized(true);  

      backend_->registerStampedPose(msg.transforms[0].header.stamp, pose_id_);
      backend_->updatePose(pose);
    }
    else
    {
      if (! online_)
        return; 

      if ( (msg.transforms[0].header.stamp - prev_stamp_).toSec() < 5.0 )
        return;

      // std::pair<int, bool> associated_id = backend_->searchAssociatedPose(msg.transforms[0].header.stamp);
      // pose_id_ = associated_id.first;  
      
      pose_id_ = backend_->incrementPoseID();
      backend_->registerStampedPose(msg.transforms[0].header.stamp, pose_id_);

      // double fMin = -4.0;
      // double fMax = 4.0;
      // // double f = (double)rand() / RAND_MAX;
      // double x = fMin + ((double)rand() / RAND_MAX) * (fMax - fMin);
      // double y = fMin + ((double)rand() / RAND_MAX) * (fMax - fMin);
      // delta = gtsam::Pose3(gtsam::Rot3::Rodrigues(0.0, 0.0, 0.0), gtsam::Point3(x, y, 0.0));
    }


    // std::cout << "GNSS callback() - ID: " << pose_id_ << ", stamp: " << msg.transforms[0].header.stamp << std::endl;

    gtsam::Key pose_key = gtsam::symbol_shorthand::X(pose_id_);       
    // pose = pose.compose(delta);
    backend_->tryInsertValue(pose_key, pose);
    backend_->addFactor(
      gtsam::PriorFactor<gtsam::Pose3>(
        pose_key, pose, noise_
      )
    );

    // backend_->insertMeasurementStamp( pose_key, msg.transforms[0].header.stamp.toSec() );

    prev_stamp_ = msg.transforms[0].header.stamp;
    backend_->isUpdated();
  }

};


} // namespace factor_handler

} // namespace backend
