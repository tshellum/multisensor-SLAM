#pragma once

/*** ROS packages ***/
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include "backend/VO_msg.h"

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

class VOHandler : public FactorHandler<const backend::VO_msg> 
{
private:
  const gtsam::noiseModel::Diagonal::shared_ptr noise_; 

  int       from_id_;
  ros::Time from_time_;
  int       num_rec_meas_;

public:
  VOHandler(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size, 
    std::shared_ptr<Backend> backend
  ) : FactorHandler(nh, topic, queue_size, backend), 
      noise_(  
        gtsam::noiseModel::Diagonal::Sigmas( 
          (gtsam::Vector(6) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished()  // rad/deg?, rad/deg?, rad/deg?, m, m, m 
        ) 
      ),
      from_id_(0), from_time_(0.0), num_rec_meas_(0)
  {
    
  }
  ~VOHandler() = default; 


  void callback(const backend::VO_msg msg)
  {
    if ( (backend_->checkNavStatus() == false) && (backend_->checkInitialized() == false) )
    {
      backend_->tryInsertValue(gtsam::symbol_shorthand::X(backend_->getPoseID()), gtsam::Pose3::identity());
      backend_->addFactor(
        gtsam::PriorFactor<gtsam::Pose3>(
          gtsam::symbol_shorthand::X(backend_->getPoseID()), gtsam::Pose3::identity(), noise_
        )
      );
      backend_->setInitialized(true);
      backend_->incrementPoseID();

      return;
    }

    if (backend_->checkInitialized() == false)
      return;
    

    Eigen::Isometry3d T_b1b2;
    tf2::fromMsg(msg.pose, T_b1b2);
    gtsam::Pose3 pose_relative(T_b1b2.matrix()); 

    std::pair<int, bool> associated_id = backend_->searchAssociatedPose(msg.header.stamp, from_time_);
    int to_id = associated_id.first;

    gtsam::Key pose_key_from = gtsam::symbol_shorthand::X(from_id_); 
    gtsam::Key pose_key_to   = gtsam::symbol_shorthand::X(to_id); 

    ROS_INFO_STREAM("vo - id: " << to_id);

    backend_->tryInsertValue(pose_key_to, pose_relative);  
    backend_->addFactor(
      gtsam::BetweenFactor<gtsam::Pose3>(
        pose_key_from, pose_key_to, pose_relative, noise_
      )
    );

    backend_->updatedPreviousRelativeTimeStamp(msg.header.stamp);
    from_id_ = to_id;
    from_time_ = msg.header.stamp;

    backend_->isUpdated();
  }

};


} // namespace factor_handler

} // namespace backend
