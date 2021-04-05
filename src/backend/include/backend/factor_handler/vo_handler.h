#pragma once

/*** ROS packages ***/
#include <tf2_eigen/tf2_eigen.h>
#include "backend/VO_msg.h"

/*** Eigen packages ***/
#include <Eigen/Dense>

/*** GTSAM packages ***/
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 
#include <gtsam/sam/BearingRangeFactor.h>

/*** Boost packages ***/
#include <boost/property_tree/ptree.hpp>

/*** Local ***/
#include "backend/factor_handler/factor_handler.h"


namespace backend
{

namespace factor_handler
{

class VOHandler : public FactorHandler<const backend::VO_msg> 
{
private:
  gtsam::noiseModel::Diagonal::shared_ptr noise_; 

  int       from_id_;
  ros::Time from_time_;

  gtsam::Pose3 pose_initial_;

public:
  VOHandler(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size, 
    std::shared_ptr<Backend> backend,
    boost::property_tree::ptree parameters = boost::property_tree::ptree()
  ) 
  : FactorHandler(nh, topic, queue_size, backend)
  , from_id_(backend_->getPoseID())
  , from_time_(0.0)
  {
    if ( parameters != boost::property_tree::ptree() )
    {
      noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        ( gtsam::Vector(6) << gtsam::Vector3::Constant(parameters.get< double >("visual_odometry.orientation_sigma")), 
                              gtsam::Vector3::Constant(parameters.get< double >("visual_odometry.position_sigma"))
        ).finished()
      );
    }
    else
    {
      noise_ = gtsam::noiseModel::Diagonal::Sigmas( 
        ( gtsam::Vector(6) << gtsam::Vector3::Constant(M_PI/18), 
                              gtsam::Vector3::Constant(0.3)
        ).finished()
      );
    }
    
    pose_initial_ = gtsam::Pose3::identity();
    if (parameters != boost::property_tree::ptree())
    {
      Eigen::Quaterniond q(parameters.get< double >("pose_origin.orientation.w"), 
                           parameters.get< double >("pose_origin.orientation.x"), 
                           parameters.get< double >("pose_origin.orientation.y"), 
                           parameters.get< double >("pose_origin.orientation.z"));
      
      Eigen::Vector3d t(parameters.get< double >("pose_origin.translation.x"), 
                        parameters.get< double >("pose_origin.translation.y"), 
                        parameters.get< double >("pose_origin.translation.z"));

      pose_initial_ = gtsam::Pose3(gtsam::Rot3(q), gtsam::Point3(t));
    }
  }
  ~VOHandler() = default; 


  void callback(const backend::VO_msg msg)
  { 
    // Insert initial
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
    

    // Add pose
    Eigen::Isometry3d T_b1b2;
    tf2::fromMsg(msg.pose, T_b1b2);
    gtsam::Pose3 pose_relative(T_b1b2.matrix()); 

    std::pair<int, bool> associated_id = backend_->searchAssociatedPose(msg.header.stamp, from_time_);
    int to_id = associated_id.first;

    gtsam::Key pose_key_from = gtsam::symbol_shorthand::X(from_id_); 
    gtsam::Key pose_key_to   = gtsam::symbol_shorthand::X(to_id); 

    ROS_INFO_STREAM("VO - from_id: " << from_id_ << ", to_id: " << to_id);
    // pose_relative.print("VO BETWEEN");

    backend_->tryInsertValue(pose_key_to, pose_relative);  
    backend_->addFactor(
      gtsam::BetweenFactor<gtsam::Pose3>(
        pose_key_from, pose_key_to, pose_relative, noise_
      )
    );

    backend_->updatedPreviousRelativeTimeStamp(msg.header.stamp);


    // // Add landmarks
    // gtsam::Pose3 pose_prev = backend_->getPoseAt(pose_key_from);
    // gtsam::Pose3 pose_cur = pose_prev.compose(pose_relative);

    // for(int landmark_it = 0; landmark_it < msg.cloud_size; landmark_it++)
    // {
    //   Eigen::Vector3d pt_eig;
    //   tf2::fromMsg(msg.cloud[landmark_it].world_point, pt_eig);
    //   gtsam::Point3 pt(pt_eig);
    //   double pt_id = msg.cloud[landmark_it].id;

    //   gtsam::Key landmark_key = gtsam::symbol_shorthand::L(pt_id);

    //   gtsam::BearingRange<gtsam::Pose3,gtsam::Point3> br;
    //   gtsam::Unit3 bearing = br.MeasureBearing(gtsam::Pose3::identity(), pt);
    //   double range = br.MeasureRange(gtsam::Pose3::identity(), pt);
    //   gtsam::noiseModel::Diagonal::shared_ptr landmark_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.3));
    //   backend_->addFactor(
    //     gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>(
    //       pose_key_to, landmark_key, bearing, range, landmark_noise
    //     )
    //   );  

    //   // Transform to world coordinates for initial value
    //   gtsam::Point3 pt_wrld = pose_cur * pt;
    //   backend_->tryInsertValue(landmark_key, pt_wrld);
    // }
    
    
    // End of iteration updates
    from_id_ = to_id;
    from_time_ = msg.header.stamp;

    backend_->isUpdated();
  }

};


} // namespace factor_handler

} // namespace backend
