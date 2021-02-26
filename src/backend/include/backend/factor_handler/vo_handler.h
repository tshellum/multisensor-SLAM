#pragma once

/*** ROS packages ***/
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Dense>

/*** GTSAM packages ***/
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 

/*** Local ***/
#include "backend/factor_handler/factor_handler.h"

namespace backend
{

namespace factor_handler
{

class VOHandler : public FactorHandler<const geometry_msgs::PoseStampedConstPtr&> 
{
private:
  const gtsam::noiseModel::Diagonal::shared_ptr noise_; 

  int       from_id_;
  ros::Time from_time_;
public:
  VOHandler(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size, 
    std::shared_ptr<Backend> backend
  ) : FactorHandler(nh, topic, queue_size, backend), 
      noise_( 
        gtsam::noiseModel::Diagonal::Sigmas( 
          ( gtsam::Vector6() 
            << gtsam::Vector3(1.0, 1.0, 1.0), // TODO: Tune
            gtsam::Vector3(1.0, 1.0, 1.0) 
          ).finished() 
        ) 
      ),
      from_id_(0), from_time_(0.0)    
  {
    
  }
  ~VOHandler() = default; 


  void callback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    Eigen::Isometry3d T_b1b2;
    tf2::fromMsg(msg->pose, T_b1b2);
    gtsam::Pose3 pose(T_b1b2.matrix()); 

    std::pair<int, bool> associated_id = backend_->searchAssociatedPose(msg->header.stamp, from_time_);
    gtsam::Key pose_key = gtsam::symbol_shorthand::X(associated_id.first); 
    
    // Values - update(): https://gtsam.org/doxygen/4.0.0/a03871.html#a47bf2a64ee131889b02049b242640226
    // Graph - rekey(): http://www.borg.cc.gatech.edu/sites/edu.borg/html/a00181.html

    gtsam::Key pose_key_from = gtsam::symbol_shorthand::X(from_id_); 
    gtsam::Key pose_key_to   = gtsam::symbol_shorthand::X(associated_id.first); 

    // if (! backend_->getValues().exists(pose_key_to))
    //   backend_->getValues().insert(pose_key_to, pose); 

    // backend_->getGraph().add(
    //   gtsam::BetweenFactor<gtsam::Pose3>(
    //     pose_key_from, pose_key_to, pose, noise_
    //   )
    // ); 

    from_id_ = associated_id.first;
    from_time_ = msg->header.stamp;
  }

};


} // namespace factor_handler

} // namespace backend
