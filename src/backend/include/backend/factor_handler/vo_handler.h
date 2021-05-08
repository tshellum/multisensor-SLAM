#pragma once

/*** ROS packages ***/
#include <tf2_eigen/tf2_eigen.h>
#include "backend/VO_msg.h"
#include "backend/VSLAM_msg.h"

/*** Eigen packages ***/
#include <Eigen/Dense>

/*** GTSAM packages ***/
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 
#include <gtsam/slam/ProjectionFactor.h>

/*** Boost packages ***/
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

/*** Local ***/
#include "backend/factor_handler/factor_handler.h"
#include "support.h"



namespace backend
{

namespace factor_handler
{

struct StereoMeasurement
{
  gtsam::Point3 landmark;
  gtsam::Point2 lfeature;
  gtsam::Point2 rfeature; 
  
  StereoMeasurement() {};

  StereoMeasurement(gtsam::Point3 new_landmark,
                    gtsam::Point2 new_lfeature,
                    gtsam::Point2 new_rfeature) 
  : landmark(new_landmark)
  , lfeature(new_lfeature)
  , rfeature(new_rfeature)
  {};
};



class VOHandler : public FactorHandler<const backend::VSLAM_msg> 
{
private:
  gtsam::Pose3 pose_initial_;
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise_; 
  const gtsam::SharedNoiseModel feature_noise_; // uncertainty of feature pos
  const gtsam::SharedNoiseModel landmark_noise_; // uncertainty of landmark pos
  gtsam::Pose3 pose_world_;

  // Between factor from-values
  int       from_id_;
  ros::Time from_time_;
  int       from_keyframe_id_;

  // Loop correspondences
  std::map<int, int> keyframe2graphID_correspondence_; // <vo keyframe id, graph pose id>

  // Calibration matrix 
  gtsam::Cal3_S2::shared_ptr K_;

  // Stereo
  gtsam::Pose3 T_lr_;
  gtsam::noiseModel::Diagonal::shared_ptr stereo_noise_;
  std::map<int, StereoMeasurement> prev_stereo_measurements_; // <landmark id, stereo measurement>

public:
  VOHandler(
    ros::NodeHandle nh, 
    const std::string& topic, 
    uint32_t queue_size, 
    std::shared_ptr<Backend> backend,
    boost::property_tree::ptree parameters = boost::property_tree::ptree(),
    boost::property_tree::ptree camera_parameters = boost::property_tree::ptree()
  ) 
  : FactorHandler(nh, topic, queue_size, backend)
  , from_id_(backend_->getPoseID())
  , from_keyframe_id_(from_id_)
  , from_time_(0.0)
  , feature_noise_( 
      gtsam::noiseModel::Isotropic::Sigma(2, 1.0)
    )
  , landmark_noise_( 
      gtsam::noiseModel::Isotropic::Sigma(3, 2.0)
    )
  , stereo_noise_ (
      gtsam::noiseModel::Diagonal::Sigmas(
        ( gtsam::Vector(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ).finished()
      )
    )
  {
    if ( parameters != boost::property_tree::ptree() )
    {
      pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        ( gtsam::Vector(6) << gtsam::Vector3::Constant(parameters.get< double >("visual_odometry.orientation_sigma")), 
                              gtsam::Vector3::Constant(parameters.get< double >("visual_odometry.position_sigma"))
        ).finished()
      );
    }
    else
    {
      pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas( 
        ( gtsam::Vector(6) << gtsam::Vector3::Constant(M_PI/18), 
                              gtsam::Vector3::Constant(0.3)
        ).finished()
      );
    }
    
    // pose_initial_ = gtsam::Pose3::identity();
    // if (parameters != boost::property_tree::ptree())
    // {
    //   Eigen::Quaterniond q(parameters.get< double >("pose_origin.orientation.w"), 
    //                        parameters.get< double >("pose_origin.orientation.x"), 
    //                        parameters.get< double >("pose_origin.orientation.y"), 
    //                        parameters.get< double >("pose_origin.orientation.z"));
      
    //   Eigen::Vector3d t(parameters.get< double >("pose_origin.translation.x"), 
    //                     parameters.get< double >("pose_origin.translation.y"), 
    //                     parameters.get< double >("pose_origin.translation.z"));

    //   pose_initial_ = gtsam::Pose3(gtsam::Rot3(q), gtsam::Point3(t));
    // }

    if (camera_parameters != boost::property_tree::ptree())
    {
      double intrinsics[9]; 

      int i = 0;
      BOOST_FOREACH(boost::property_tree::ptree::value_type& v, camera_parameters.get_child("camera_left.intrinsics"))
        intrinsics[i++] = v.second.get_value<double>(); 

      K_ = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(intrinsics[0], intrinsics[4], intrinsics[1], intrinsics[2], intrinsics[5]));

      // double t[3]; 
      // double R[9]; 

      // i = 0;
      // BOOST_FOREACH(boost::property_tree::ptree::value_type &v, camera_parameters.get_child("stereo.rotation"))
      //   R[i++] = v.second.get_value<double>(); 

      // i = 0;
      // BOOST_FOREACH(boost::property_tree::ptree::value_type &v, camera_parameters.get_child("stereo.translation"))
      //   t[i++] = v.second.get_value<double>(); 
            
      // T_lr_ = gtsam::Pose3(gtsam::Rot3(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]), gtsam::Point3(t[0], t[1], t[2]));
    }

  }
  ~VOHandler() = default; 


  void callback(const backend::VSLAM_msg msg)
  { 
    if (backend_->checkInitialized() == false)
      return;
    

    // Add pose
    Eigen::Isometry3d T_b1b2;
    tf2::fromMsg(msg.pose, T_b1b2);
    gtsam::Pose3 pose_relative(T_b1b2.matrix()); 

    std::pair<int, bool> associated_id = backend_->searchAssociatedPose(msg.header.stamp, from_time_);
    int to_id = associated_id.first;

    // ROS_INFO_STREAM("VO() - from_id: " << from_id_ << ", to_id: " << to_id << ", stamp: " << msg.header.stamp );

    gtsam::Key pose_key_from = gtsam::symbol_shorthand::X(from_id_); 
    gtsam::Key pose_key_to   = gtsam::symbol_shorthand::X(to_id); 

    pose_world_ = backend_->getPoseAt(pose_key_from);
    pose_world_ = pose_world_.compose(pose_relative);

    backend_->tryInsertValue(pose_key_to, pose_world_);  
    backend_->addFactor(
      gtsam::BetweenFactor<gtsam::Pose3>(
        pose_key_from, pose_key_to, pose_relative, pose_noise_
      )
    );

    // Update keyframe --> pose id in graph
    if ( msg.is_keyframe.data )
      keyframe2graphID_correspondence_[msg.keyframe_id] = to_id;

    // Insert loop closure
    if ( msg.loop_found.data )
    {
      Eigen::Isometry3d T_loop;
      tf2::fromMsg(msg.pose_loop, T_loop);
      gtsam::Pose3 pose_loop(T_loop.matrix()); 

      int loop_to_id = keyframe2graphID_correspondence_[msg.match_id];
      gtsam::Key loop_key_from = gtsam::symbol_shorthand::X(to_id); 
      gtsam::Key loop_key_to   = gtsam::symbol_shorthand::X(loop_to_id); 

      backend_->addFactor(
        gtsam::BetweenFactor<gtsam::Pose3>(
          loop_key_from, loop_key_to, pose_loop, pose_noise_
        )
      );

      gtsam::Pose3 pose_loop_to = backend_->getPoseAt(loop_key_to);
      backend_->addFactor(
        gtsam::PriorFactor<gtsam::Pose3>(
          loop_key_to, pose_loop_to, gtsam::noiseModel::Constrained::All(6)
        )
      );

      backend_->markUpdateWithLoop();
    }

    // Add landmarks
    if (msg.is_keyframe.data || (from_keyframe_id_ == 0))
    {
      gtsam::Key pose_key_from_kf = gtsam::symbol_shorthand::X(from_keyframe_id_); 
      gtsam::Pose3 sensorTbody = gtsam::Pose3( gtsam::Rot3(
        0,  0, 1,
       -1,  0, 0,
        0, -1, 0
      ), gtsam::Point3()); // CAM --> body

      gtsam::Pose3 bodyTsensor = gtsam::Pose3( gtsam::Rot3(
        0, -1,  0,
        0,  0, -1,
        1,  0,  0
      ), gtsam::Point3()); // body --> CAM


      std::map<int, StereoMeasurement> cur_stereo_measurements; // <landmark id, stereo measurement>

      // int number_of_landmarks_to_add = std::min(msg.landmark_size, 50);
      int num_inserted = 0;
      for(int idx = 0; idx < msg.landmark_size; idx++)
      {
        std::pair<Eigen::Vector3d, int> landmark_wID = fromPt3DMsg( msg.landmarks[idx] );
        std::pair<Eigen::Vector2d, int> lfeature_wID = fromPt2DMsg( msg.lfeatures[idx] );
        std::pair<Eigen::Vector2d, int> rfeature_wID = fromPt2DMsg( msg.rfeatures[idx] );
        int landmark_id = lfeature_wID.second;
        gtsam::Key landmark_key = gtsam::symbol_shorthand::L(landmark_id);
        gtsam::Point3 landmark = gtsam::Point3(landmark_wID.first);        
        landmark = pose_world_.translation() + pose_world_.rotation()*landmark;
        gtsam::Point2 lfeature = gtsam::Point2(lfeature_wID.first);
        gtsam::Point2 rfeature = gtsam::Point2(rfeature_wID.first);
        

        // Insert measurements
        cur_stereo_measurements[landmark_id] = StereoMeasurement(landmark, lfeature, rfeature);

        std::map<int, StereoMeasurement>::iterator landmark_it = prev_stereo_measurements_.find(landmark_id);
        if ( (landmark_it != prev_stereo_measurements_.end()) && (num_inserted < 50) ) 
        {
          num_inserted++;

          if ( backend_->tryInsertValue(landmark_key, prev_stereo_measurements_[landmark_id].landmark) )
          {
            backend_->addFactor(
              gtsam::PriorFactor<gtsam::Point3>(
                landmark_key, prev_stereo_measurements_[landmark_id].landmark, landmark_noise_
              )
            );

            backend_->addFactor(
              gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3>(
                prev_stereo_measurements_[landmark_id].lfeature, feature_noise_, pose_key_from_kf, landmark_key, K_, sensorTbody
              )
            );
          }

          backend_->addFactor(
            gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3>(
              lfeature, feature_noise_, pose_key_to, landmark_key, K_, sensorTbody
            )
          );
        }
      }

      prev_stereo_measurements_ = cur_stereo_measurements;
      from_keyframe_id_ = to_id;

      // std::cout << "\nAdded " << num_inserted << " landmarks.." << std::endl;
    }
    

    // End of iteration updates
    from_id_ = to_id;
    from_time_ = msg.header.stamp;

    backend_->isUpdated();
  }

};


} // namespace factor_handler

} // namespace backend




