#pragma once

/*** ROS packages ***/
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/PoseStamped.h>

/*** Eigen packages ***/
#include <Eigen/Dense>

/*** GTSAM packages ***/
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 

/*** Boost packages ***/
#include <boost/property_tree/ptree.hpp>

/*** Local ***/
#include "backend/factor_handler/factor_handler.h"


namespace backend
{

namespace factor_handler
{

class LidarOdometryHandler : public FactorHandler<const geometry_msgs::PoseStamped> 
{
private:
  const gtsam::noiseModel::Diagonal::shared_ptr noise_; 

  int       from_id_;
  ros::Time from_time_;

  gtsam::Pose3 pose_initial_;

public:
  LidarOdometryHandler(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size, 
    std::shared_ptr<Backend> backend,
    boost::property_tree::ptree parameters = boost::property_tree::ptree()
  ) 
  : FactorHandler(nh, topic, queue_size, backend)
  , noise_(  
      gtsam::noiseModel::Diagonal::Sigmas( 
        (gtsam::Vector(6) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished()  // rad/deg?, rad/deg?, rad/deg?, m, m, m 
      ) 
    )
  , from_id_(0)
  , from_time_(0.0)
  {
    pose_initial_ = gtsam::Pose3::identity();
    if (parameters != boost::property_tree::ptree())
    {
      Eigen::Quaterniond q(parameters.get< double >("pose.orientation.w"), 
                           parameters.get< double >("pose.orientation.x"), 
                           parameters.get< double >("pose.orientation.y"), 
                           parameters.get< double >("pose.orientation.z"));
      
      Eigen::Vector3d t(parameters.get< double >("pose.translation.x"), 
                        parameters.get< double >("pose.translation.y"), 
                        parameters.get< double >("pose.translation.z"));

      pose_initial_ = gtsam::Pose3(gtsam::Rot3(q), gtsam::Point3(t));
      

      Eigen::Quaterniond q_cor;
      q_cor = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(-M_PI/5.2, Eigen::Vector3d::UnitZ());

      pose_initial_ = pose_initial_.compose(gtsam::Pose3(gtsam::Rot3(q_cor),
                                         gtsam::Point3()));
    }
  }
  ~LidarOdometryHandler() = default; 


  void callback(const geometry_msgs::PoseStamped msg)
  { 
    if ( (backend_->checkNavStatus() == false) && (backend_->checkInitialized() == false) ) // GNSS is offline and the graph is not yet initialized by any module --> initialize
    {
      backend_->tryInsertValue(gtsam::symbol_shorthand::X(backend_->getPoseID()), pose_initial_);
      backend_->addFactor(
        gtsam::PriorFactor<gtsam::Pose3>(
          gtsam::symbol_shorthand::X(backend_->getPoseID()), pose_initial_, noise_
        )
      );
      backend_->setInitialized(true);

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

    ROS_INFO_STREAM("Lidar Odomtery - from_id: " << from_id_ << ", to_id: " << to_id);

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
