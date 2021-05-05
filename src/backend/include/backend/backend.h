#pragma once 

/*** ROS packages ***/
#include <ros/ros.h> 
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/*** GTSAM packages ***/
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
// #include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include "gtsam/nonlinear/Marginals.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/ImuBias.h>

/*** Local ***/ 
#include "support.h"
#include "backend/motion_model.h"

/*** Standard library ***/ 
#include <fstream>
#include <time.h>


enum { NONE = 0, REMOVE_IMU = 1, INVALID_MOTION = 2, INDETERMINANT_LINEAR_SYSTEM = 3 };


namespace backend
{

class Backend
{
private:
  // Nodes
  ros::NodeHandle nh_;
  ros::Publisher world_pose_pub_;
  ros::Timer optimize_timer_;  // Spun node
  const int buffer_size_;

  // File
  std::string result_path_;
  std::string graph_filename_;

  // ISAM2
  int num_opt_;
  gtsam::ISAM2 isam2_; 
  gtsam::ISAM2Params isam2_params_; 
  gtsam::Values current_estimate_; 

  // Graph 
  bool updated_; 
  gtsam::Values new_values_; 
  gtsam::NonlinearFactorGraph new_factors_;

  // Association
  ros::Time time_prev_pose_relative_;
  double association_threshold_; // in seconds
  std::map<ros::Time, int> stamped_pose_ids_;
  bool update_contains_loop_;
  
  // Current state
  int pose_id_;
  gtsam::Pose3 pose_;
  gtsam::Vector3 velocity_;
  gtsam::imuBias::ConstantBias bias_;

  // Measurement states
  bool nav_status_;  // is online
  bool initialized_; // initialized pos by nav

  // Motion model
  MotionModel motion_model_;

public:
  Backend() 
  : pose_id_(0)
  , num_opt_(20)
  , time_prev_pose_relative_(ros::Time(0.0))
  , initialized_(false)
  , updated_(false)
  , nav_status_(false)
  , association_threshold_(0.1)
  , pose_(gtsam::Pose3::identity())
  , velocity_(gtsam::Vector3(0.0, 0.0, 0.0))
  , graph_filename_("graph.dot")
  , result_path_(ros::package::getPath("backend") + "/../../results/")
  , buffer_size_(1000)
  , optimize_timer_(nh_.createTimer(ros::Duration(0.1), &Backend::callback, this))
  , world_pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("/backend/pose", buffer_size_))
  , update_contains_loop_(false)
  {
    double lag = 2.0;
   
    isam2_params_.relinearizeThreshold = 0.1;
    isam2_params_.relinearizeSkip = 10;
    isam2_params_.enableRelinearization = true; 
    isam2_params_.evaluateNonlinearError = false; 
    isam2_params_.factorization = gtsam::ISAM2Params::CHOLESKY; 
    isam2_params_.cacheLinearizedFactors = true;
    isam2_ = gtsam::ISAM2(isam2_params_);
  };

  ~Backend() 
  {
    isam2_.saveGraph(result_path_ + graph_filename_);
  }


  int getPoseID() const                   { return pose_id_; }
  int getNewestPoseID() const             { return stamped_pose_ids_.rbegin()->second; }
  ros::Time getNewestPoseTime() const     { return stamped_pose_ids_.rbegin()->first; }
  gtsam::Values& getValues()              { return new_values_;  }
  gtsam::NonlinearFactorGraph& getGraph() { return new_factors_; }
  gtsam::Pose3 getPose()                  { return pose_; }
  gtsam::Pose3 getPoseAt(gtsam::Key key);
  gtsam::Point3 getPointAt(gtsam::Key key);
  gtsam::Vector3 getVelocity()            { return velocity_; }
  gtsam::imuBias::ConstantBias getBias()  { return bias_; }
  gtsam::ISAM2& getiSAM2()                { return isam2_; }
  bool checkInitialized()                 { return initialized_; }
  bool checkNavStatus()                   { return nav_status_; }
  std::map<ros::Time, int> getStampedPoseIDs() { return stamped_pose_ids_; }
  MotionModel getMotionModel()            { return motion_model_; }

  int  incrementPoseID()                                { return ++pose_id_; }
  void registerStampedPose(ros::Time stamp, int id)     { stamped_pose_ids_[stamp] = id; }
  void setInitialized(bool status)                      { initialized_ = status; }
  void isUpdated()                                      { updated_ = true; }
  void updatedPreviousRelativeTimeStamp(ros::Time time) { time_prev_pose_relative_ = time; }
  void updateNavStatus(bool status)                     { nav_status_ = status; }
  void updatePose(gtsam::Pose3 pose)                    { pose_ = pose; }
  void updateVelocity(gtsam::Vector3 velocity)          { velocity_ = velocity; }
  void updateBias(gtsam::imuBias::ConstantBias bias)    { bias_ = bias; }
  void markUpdateWithLoop()                             { update_contains_loop_ = true; }

  geometry_msgs::PoseStamped generateMsg();
  void callback(const ros::TimerEvent& event);

  bool valueExist(gtsam::Key key);

  template <typename Value>
  bool tryInsertValue(gtsam::Key key, Value value);

  template <typename Value>
  void forceInsertValue(gtsam::Key key, Value value);

  bool tryEraseValue(gtsam::Key key);

  template <typename Value>
  bool tryGetEstimate(gtsam::Key key, gtsam::Values result, Value& estimate);

  template <typename FactorType>
  void addFactor(FactorType factor) { new_factors_.add(factor); }

  std::pair<int, bool> searchAssociatedPose(ros::Time pose_stamp, ros::Time prev_pose_stamp = ros::Time(0.0));

  void clearOldAssocations(double interval = 1.0);
  ros::Time findPoseStamp(int pose_id);
  bool isValidMotion(gtsam::Pose3 pose_relative, double x_thresh=-0.5, double yaw_thresh=M_PI/9, double pitch_thresh=M_PI/18, double roll_thresh=M_PI/18);
};



void Backend::callback(const ros::TimerEvent& event)
{
  if (updated_)
  {
    ros::Time tic = ros::Time::now();
    
    updated_ = false;
    int newest_pose_id = stamped_pose_ids_.rbegin()->second; // There is a processing delay with VO - Rather use newest stamped pose (perhaps gnss)

    // new_values_.print("----- Values -----");
    // new_factors_.print("----- New factors -----");


    // Optimize   
    bool valid_motion = false;
    bool invalid_optimization = true;
    int error_correction = NONE;
    gtsam::ISAM2 isam2_error_prone = isam2_; // Save before optimize graph
    gtsam::Values result;
    int indeterminant_index;
    gtsam::FactorIndices remove_indices;
    while (invalid_optimization)
    {
      try
      {
        isam2_.update(new_factors_, new_values_, remove_indices);
        for (int i = 0; i < num_opt_; ++i)
          isam2_.update(); 


        result = isam2_.calculateBestEstimate();
        gtsam::Pose3 pose_previous = pose_;
        gtsam::Pose3 pose_current;
        tryGetEstimate(gtsam::symbol_shorthand::X(newest_pose_id), result, pose_current);

        // Ensure valid motion if all conditions are met
        gtsam::Pose3 pose_relative = pose_previous.between(pose_current);

        if ( isValidMotion(pose_relative, -0.5, M_PI / 9, M_PI / 18, M_PI / 18) )
          error_correction = NONE;
        else
        {
          if (! ( (pose_relative.x() >= -0.5)
               || ( (pose_relative.x() >= pose_relative.y())
                 && (pose_relative.x() >= pose_relative.z()) ) ) )
          {
            if (pose_relative.x() <= -0.5)
              std::cout << "x: " << pose_relative.x() << std::endl;
            
            if (pose_relative.x() <= pose_relative.y()) 
              std::cout << "x=" << pose_relative.x() << " > y=" << pose_relative.y() << std::endl;
            
            if (pose_relative.x() <= pose_relative.z()) 
              std::cout << "x=" << pose_relative.x() << " > z=" << pose_relative.z() << std::endl;
          }
          
          double yaw_thresh = M_PI / 9;
          double pitch_thresh = M_PI / 18;
          double roll_thresh = M_PI / 18;

          if (std::abs(pose_relative.rotation().yaw()) > yaw_thresh)
            std::cout << "YAW=" << std::abs(pose_relative.rotation().yaw()) << " > " << yaw_thresh << std::endl;

          if (std::abs(pose_relative.rotation().pitch()) > pitch_thresh)
            std::cout << "PITCH=" << std::abs(pose_relative.rotation().pitch()) << " > " << pitch_thresh << std::endl;

          if (std::abs(pose_relative.rotation().roll()) > roll_thresh)
            std::cout << "ROLL=" << std::abs(pose_relative.rotation().roll()) << " > " << roll_thresh << std::endl;

          if (update_contains_loop_)
            std::cout << "Loop closure? : " << update_contains_loop_ << std::endl;


          // std::cout << "- Velocity: " << result.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(newest_pose_id)).transpose() << std::endl;
          // std::cout << "- Bias: ";
          // result.at<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(newest_pose_id)).print();

          error_correction = (error_correction == REMOVE_IMU ? NONE : REMOVE_IMU );
        }
      }
      catch(gtsam::IndeterminantLinearSystemException e)
      {
        indeterminant_index = e.nearbyVariable();
        std::cerr << "\nCAPTURED THE EXCEPTION: " << e.nearbyVariable() << std::endl;
        // int remove_idx = ( e.nearbyVariable() > gtsam::symbol_shorthand::V(0) ? 
        //     e.nearbyVariable() - gtsam::symbol_shorthand::V(0) : e.nearbyVariable() - gtsam::symbol_shorthand::B(0) ); 

        error_correction = (error_correction == REMOVE_IMU ? NONE : REMOVE_IMU );
      }


      switch (error_correction)
      {
      case REMOVE_IMU:
      {
        std::cout << "\nREMOVE IMU" << std::endl;
        isam2_ = isam2_error_prone;
        
        // Typically the IMU is the issue --> remove IMUfactors
        for ( int i = 0; i < new_factors_.size(); )
        {
          if ( new_factors_.at(i)->size() == 6 )
          {
            tryEraseValue(new_factors_.at(i)->keys()[3]); // Velocity to key
            tryEraseValue(new_factors_.at(i)->keys()[5]); // Bias to key

            new_factors_.erase( new_factors_.begin() + i ); // IMUfactor
          }
          else
            i++;
        }

        invalid_optimization = true;
        break;
      }
      case INVALID_MOTION:
      {
        isam2_ = isam2_error_prone;
        // Fix
        invalid_optimization = true;
        break;
      }
      case INDETERMINANT_LINEAR_SYSTEM:
      {
        gtsam::VariableIndex key2factor_index = isam2_.getVariableIndex();
        remove_indices = key2factor_index[indeterminant_index];
        break;
      }
      case NONE:
      default:
        tryGetEstimate(gtsam::symbol_shorthand::X(newest_pose_id), result, pose_);
        tryGetEstimate(gtsam::symbol_shorthand::V(newest_pose_id), result, velocity_);
        tryGetEstimate(gtsam::symbol_shorthand::B(newest_pose_id), result, bias_);
        current_estimate_ = result;
        invalid_optimization = false;
        break;
      }
    }


    // End of iteration updates
    world_pose_pub_.publish(generateMsg()); // TODO: Use latest stamp from map

    // Reset parameters
    new_factors_.resize(0);
    new_values_.clear();
    clearOldAssocations();
    update_contains_loop_ = false;

    ros::Time toc = ros::Time::now();
    printSummary((toc - tic).toSec(),
                  Eigen::Affine3d{pose_.matrix()} );
  }
}



// void Backend::callback(const ros::TimerEvent& event)
// {
//   if (updated_)
//   {
//     ros::Time tic = ros::Time::now();
    
//     updated_ = false;
//     int newest_pose_id = stamped_pose_ids_.rbegin()->second; // There is a processing delay with VO - Rather use newest stamped pose (perhaps gnss)

//     int m = 0;
//     bool remove_IMUfactor = false;
//     bool valid_estimate = false;

//     // new_values_.print("----- Values -----");
//     // new_factors_.print("----- New factors -----");


//     // Optimize   
//     gtsam::ISAM2 isam2_error_prone = isam2_; // Save before optimize graph

//     // ROS_INFO("OPTIMIZING");

//     while (! valid_estimate)
//     {
//       m++;
//       isam2_ = isam2_error_prone;

//       if (remove_IMUfactor)
//       {
//         // ROS_INFO("Remove IMU");

//         // Typically the IMU is the issue --> remove IMUfactors
//         for ( int i = 0; i < new_factors_.size(); )
//         {
//           if ( new_factors_.at(i)->size() == 6 )
//           {
//             tryEraseValue(new_factors_.at(i)->keys()[3]); // Velocity to key
//             tryEraseValue(new_factors_.at(i)->keys()[5]); // Bias to key

//             new_factors_.erase( new_factors_.begin() + i ); // IMUfactor
//           }
//           else
//             i++;
//         }

//         remove_IMUfactor = false;
//         ROS_INFO("Removed IMU");
//       }

//       try
//       {
//         // ROS_INFO_STREAM("Initial update OPTIMIZING, it: " << m);

//         isam2_.update(new_factors_, new_values_);
//         for (int i = 0; i < num_opt_; ++i)
//           isam2_.update(); 

//         // ROS_INFO("Initial update OPTIMIZED");
//       }
//       catch(gtsam::IndeterminantLinearSystemException e)
//       {
//         std::cerr << "\nCAPTURED THE EXCEPTION: " << e.nearbyVariable() << std::endl;
//         if (! remove_IMUfactor)
//         {
//           remove_IMUfactor = true;
//           continue;
//         }
//       }
   
//       gtsam::Pose3 pose_pre_optimize = pose_;

//       // ROS_INFO_STREAM("ISAM SIZE: "<< isam2_.size() << " - Get estimate");

//       // Get updated values
//       gtsam::Values result = isam2_.calculateBestEstimate();
//       tryGetEstimate(gtsam::symbol_shorthand::X(newest_pose_id), result, pose_);

//       // Ensure valid motion - Erroneous if all criterias are true
//       gtsam::Pose3 between = pose_pre_optimize.between(pose_);
//       if ( (isam2_.size() > 5) 
//         && ( (between.translation().norm() > 5)
//           || (between.x() < between.y())
//           || (between.x() < between.z())
//           || (std::abs(between.rotation().yaw()) > M_PI / 18) 
//           || (std::abs(between.rotation().roll()) > M_PI / 36) 
//           || (std::abs(between.rotation().pitch()) > M_PI / 36) )
//         && (! update_contains_loop_)
//         && (m < 3)
//       )
//       {
//         ROS_INFO("ERROR motion");

//         double yaw = between.rotation().yaw();
//         if (std::abs(yaw) > (M_PI / 9))
//           ROS_INFO_STREAM("YAW: " << yaw * (180.0/M_PI));

//         double pitch = between.rotation().pitch();
//         if (std::abs(pitch) > (M_PI / 18))
//           ROS_INFO_STREAM("PITCH: " << pitch * (180.0/M_PI));

//         double roll = between.rotation().roll();
//         if (std::abs(roll) > (M_PI / 18))
//           ROS_INFO_STREAM("ROLL: " << roll * (180.0/M_PI));

//         double norm = between.translation().norm();
//         if (std::abs(norm) > 3)
//           ROS_INFO_STREAM("NORM: " << norm);

//         pose_ = pose_pre_optimize;
//         remove_IMUfactor = true;
//         continue;
//       }

//       tryGetEstimate(gtsam::symbol_shorthand::V(newest_pose_id), result, velocity_);
//       tryGetEstimate(gtsam::symbol_shorthand::B(newest_pose_id), result, bias_);
//       current_estimate_ = result;

//       // ROS_INFO("GOTTEN");

//       valid_estimate = true;
//       update_contains_loop_ = false;
//     }
//     // ROS_INFO("OPTIMIZED");


//     // End of iteration updates
//     world_pose_pub_.publish(generateMsg()); // TODO: Use latest stamp from map

//     // Reset parameters
//     new_factors_.resize(0);
//     new_values_.clear();
//     clearOldAssocations();

//     ros::Time toc = ros::Time::now();
//     printSummary((toc - tic).toSec(),
//                   Eigen::Affine3d{pose_.matrix()} );
//   }
// }




// void Backend::callback(const ros::TimerEvent& event)
// {
//   if (updated_)
//   {
//     ros::Time tic = ros::Time::now();
    
//     updated_ = false;
//     int newest_pose_id = stamped_pose_ids_.rbegin()->second; // There is a processing delay with VO - Rather use newest stamped pose (perhaps gnss)

//     int m = 0;
//     bool remove_IMUfactor = false;
//     bool valid_motion = false;

//     // new_values_.print("----- Values -----");
//     // new_factors_.print("----- New factors -----");

//     // Optimize   
//     gtsam::ISAM2 isam2_error_prone = isam2_; // Save before optimize graph


//     ROS_INFO("OPTIMIZING");

//     try
//     {
//       ROS_INFO("Initial update OPTIMIZING");

//       isam2_.update(new_factors_, new_values_);
//       for (int i = 0; i < num_opt_; ++i)
//         isam2_.update(); 

//       ROS_INFO("Initial update OPTIMIZED");
//     }
//     catch(gtsam::IndeterminantLinearSystemException e)
//     {
//       std::cerr << "\nCAPTURED THE EXCEPTION: " << e.nearbyVariable() << std::endl;
//       isam2_ = isam2_error_prone;
//       remove_IMUfactor = true;
//     }
  
//     while (! valid_motion)
//     {
//       ROS_INFO_STREAM("Updating, it " << m++);

//       if (remove_IMUfactor)
//       {
//         ROS_INFO("Remove IMU");

//         // Typically the IMU is the issue --> remove IMUfactors
//         for ( int i = 0; i < new_factors_.size(); )
//         {
//           if ( new_factors_.at(i)->size() == 6 )
//           {
//             tryEraseValue(new_factors_.at(i)->keys()[3]); // Velocity to key
//             tryEraseValue(new_factors_.at(i)->keys()[5]); // Bias to key

//             new_factors_.erase( new_factors_.begin() + i ); // IMUfactor
//           }
//           else
//             i++;
//         }

//         remove_IMUfactor = false;
//         ROS_INFO("Removed IMU");
//       }

//       int k = 0;
//       bool valid_update = false;
//       gtsam::FactorIndices remove_indices;
//       while (! valid_update)
//       {
//         ROS_INFO_STREAM("Updating after issue, it " << k);

//         isam2_ = isam2_error_prone;

//         try
//         {
//           ROS_INFO("Issue update OPTIMIZING");

//           isam2_.update(new_factors_, new_values_, remove_indices);
//           for (int i = 0; i < num_opt_; ++i)
//             isam2_.update(); 

//           ROS_INFO("Issue update OPTIMIZED");

//           valid_update = true;
//         }
//         catch(gtsam::IndeterminantLinearSystemException e)
//         {
//           int remove_idx = ( e.nearbyVariable() > gtsam::symbol_shorthand::V(0) ? 
//             e.nearbyVariable() - gtsam::symbol_shorthand::V(0) : e.nearbyVariable() - gtsam::symbol_shorthand::B(0) ); 
          
//           std::cerr << "CAPTURED THE EXCEPTION with index: " << remove_idx << std::endl;

//           tryEraseValue(gtsam::symbol_shorthand::V(remove_idx));
//           tryEraseValue(gtsam::symbol_shorthand::B(remove_idx));
                    
//           gtsam::VariableIndex key2factor_index = isam2_.getVariableIndex();
//           remove_indices = key2factor_index[e.nearbyVariable()];
//         }

//         if (k++ > 2)
//           break;
//       }


//       gtsam::Pose3 pose_pre_optimize = pose_;

//       ROS_INFO_STREAM("ISAM SIZE: "<< isam2_.size() << "Get estimate");

//       // Get updated values
//       gtsam::Values result = isam2_.calculateBestEstimate();
//       tryGetEstimate(gtsam::symbol_shorthand::X(newest_pose_id), result, pose_);

//       gtsam::Pose3 between = pose_pre_optimize.between(pose_);

//       // if ( (isam2_.size() > 5) 
//       //   && ((between.translation()).norm() > 10)
//       //   && ((between.translation()).norm() > 10)
//       //   && ((between.translation()).norm() > 10)
//       //   && ((between.translation()).norm() > 10) )

//       // double yaw = between.rotation().yaw();
//       // ROS_INFO_STREAM("YAW: " << yaw);
      
//       if ( (isam2_.size() > 5) 
//         && ((between.translation()).norm() > 10) 
//         && (m < 2))
//       {
//         ROS_INFO("ERROR motion");

//         pose_ = pose_pre_optimize;
//         remove_IMUfactor = true;
//         continue;
//       }

//       tryGetEstimate(gtsam::symbol_shorthand::V(newest_pose_id), result, velocity_);
//       tryGetEstimate(gtsam::symbol_shorthand::B(newest_pose_id), result, bias_);
//       current_estimate_ = result;

//       ROS_INFO("GOTTEN");

//       valid_motion = true;
//     }
//     ROS_INFO("OPTIMIZED");


//     // End of iteration updates
//     world_pose_pub_.publish(generateMsg()); // TODO: Use latest stamp from map

//     // Reset parameters
//     new_factors_.resize(0);
//     new_values_.clear();
//     clearOldAssocations();
//     // imu_.clearOldMeasurements(); 

//     ros::Time toc = ros::Time::now();
//     printSummary((toc - tic).toSec(),
//                   Eigen::Affine3d{pose_.matrix()} );
//   }
// }





// void Backend::callback(const ros::TimerEvent& event)
// {
//   if (updated_)
//   {
//     ros::Time tic = ros::Time::now();
    
//     updated_ = false;
//     int newest_pose_id = stamped_pose_ids_.rbegin()->second; // There is a processing delay with VO - Rather use newest stamped pose (perhaps gnss)

//     // new_values_.print("----- Values -----");
//     // new_factors_.print("----- New factors -----");

//     // Optimize   
//     gtsam::ISAM2 isam2_error_prone = isam2_; // Save before optimize graph

//     // for ( int i = 0; i < isam2_.size(); i++ )
//     // {
//     //   std::cout << "\n\nisam2_.at(" << i << ")->print(): " << std::endl;
//     //   isam2_.getFactorsUnsafe().at(i)->print();
//     // }

//     // int j = 0;
//     // bool updated = false;
//     // gtsam::FactorIndices remove_indices;
//     // while (! updated)
//     // {
//     //   bool exists = false;

//     //   try
//     //   {
//     //     isam2_ = isam2_error_prone;
//     //     isam2_.update(new_factors_, new_values_);
//     //     isam2_.update(new_factors_, gtsam::Values(), remove_indices);
//     //     for (int i = 0; i < num_opt_; ++i)
//     //       isam2_.update(); 

//     //     updated = true;
//     //   }
//     //   catch(gtsam::IndeterminantLinearSystemException e){
//     //     std::cerr << "\nCAPTURED THE EXCEPTION: " << e.nearbyVariable() << std::endl;

//     //     int remove_idx = ( e.nearbyVariable() > gtsam::symbol_shorthand::V(0) ? 
//     //       e.nearbyVariable() - gtsam::symbol_shorthand::V(0) : e.nearbyVariable() - gtsam::symbol_shorthand::B(0) ); 
        
//     //     std::cout << "Remove index: " << remove_idx << std::endl;

//     //     // tryEraseValue(gtsam::symbol_shorthand::V(remove_idx));
//     //     // tryEraseValue(gtsam::symbol_shorthand::B(remove_idx));
//     //     tryEraseValue(gtsam::symbol_shorthand::V(newest_pose_id));
//     //     tryEraseValue(gtsam::symbol_shorthand::B(newest_pose_id));
        
        
//     //     gtsam::VariableIndex key2factor_index_map = isam2_.getVariableIndex();
//     //     remove_indices = key2factor_index_map[newest_pose_id];

//     //     // remove_indices = key2factor_index_map[e.nearbyVariable()];
//     //     // remove_indices.insert(remove_indices.end(), key2factor_index_map[newest_pose_id].begin(), key2factor_index_map[newest_pose_id].end());

//     //     if (j++ > 3)
//     //       break;
//     //   }
//     // }


//     ROS_INFO("OPTIMIZING");

//     int m = 0;
//     bool remove_IMUfactor = false;
//     bool valid_motion = false;
//     while (! valid_motion)
//     {
//       ROS_INFO_STREAM("Updating, it " << m++);

//       try
//       {
//         ROS_INFO("Initial update OPTIMIZING");

//         isam2_.update(new_factors_, new_values_);
//         for (int i = 0; i < num_opt_; ++i)
//           isam2_.update(); 

//         ROS_INFO("Initial update OPTIMIZED");
//       }
//       catch(gtsam::IndeterminantLinearSystemException e)
//       {
//         std::cerr << "\nCAPTURED THE EXCEPTION: " << e.nearbyVariable() << std::endl;
//         remove_IMUfactor = true;
//       }
//         // for ( boost::shared_ptr<gtsam::NonlinearFactor> factor : new_factors_ )
//         // {
//         //   // factor->print();

//         //   std::cout << "- Num Keys: " << factor->size() << std::endl;

//         //   if ( factor->size() == 6 )
//         //   {
//         //     for ( int j = 0; j < factor->size(); j++ )
//         //       std::cout << "- Key[" << j << "]: " << factor->keys()[j] << std::endl;

//         //     new_factors_.erase( factor );
//         //   }
//         // }

//         // std::cout << "---------------------------------------------------" << std::endl;
//         // std::cout << "Size new factors before: " << new_factors_.size() << std::endl;

//         // for ( boost::shared_ptr<gtsam::NonlinearFactor> factor : new_factors_ )
//         //   factor->print();


//         // gtsam::NonlinearFactorGraph::iterator factor_iterator = new_factors_.begin();
//         // int n = 0;
//         // for (factor_iterator; factor_iterator < new_factors_.end();)
//         // {
//         //   std::cout << "n: " << n++ << std::endl;

//         //   if ( new_factors_.at(n)->size() == 6 )
//         //     factor_iterator = new_factors_.erase( factor_iterator );
//         //   else
//         //   {
//         //     n++;
//         //     factor_iterator++;
//         //   }
//         // }

//       if (remove_IMUfactor)
//       {
//         ROS_INFO("Remove IMU");

//         // Typically the IMU is the issue --> remove IMUfactors
//         for ( int i = 0; i < new_factors_.size(); )
//         {
//           if ( new_factors_.at(i)->size() == 6 )
//           {
//             tryEraseValue(new_factors_.at(i)->keys()[3]); // Velocity to key
//             tryEraseValue(new_factors_.at(i)->keys()[5]); // Bias to key

//             new_factors_.erase( new_factors_.begin() + i ); // IMUfactor
//           }
//           else
//             i++;
//         }

//         remove_IMUfactor = false;
//         ROS_INFO("Removed IMU");
//       }

//       int k = 0;
//       bool valid_update = false;
//       gtsam::FactorIndices remove_indices;
//       while (! valid_update)
//       {
//         ROS_INFO_STREAM("Updating after issue, it " << k);

//         isam2_ = isam2_error_prone;

//         try
//         {
//           ROS_INFO("Issue update OPTIMIZING");

//           isam2_.update(new_factors_, new_values_, remove_indices);
//           for (int i = 0; i < num_opt_; ++i)
//             isam2_.update(); 

//           ROS_INFO("Issue update OPTIMIZED");

//           valid_update = true;
//         }
//         catch(gtsam::IndeterminantLinearSystemException e)
//         {
//           int remove_idx = ( e.nearbyVariable() > gtsam::symbol_shorthand::V(0) ? 
//             e.nearbyVariable() - gtsam::symbol_shorthand::V(0) : e.nearbyVariable() - gtsam::symbol_shorthand::B(0) ); 
          
//           std::cerr << "CAPTURED THE EXCEPTION with index: " << remove_idx << std::endl;

//           tryEraseValue(gtsam::symbol_shorthand::V(remove_idx));
//           tryEraseValue(gtsam::symbol_shorthand::B(remove_idx));
                    
//           gtsam::VariableIndex key2factor_index = isam2_.getVariableIndex();
//           remove_indices = key2factor_index[e.nearbyVariable()];
//         }

//         if (k++ > 2)
//           break;
//       }


        



//         // std::cout << "Size new factors before: " << new_factors_.size() << std::endl;

      

//         // std::cout << "Size new factors after: " << new_factors_.size() << std::endl;


//         // for ( int i = 0; i < new_factors_.size(); i++ )
//         // {
//         //   std::cout << "\n\nnew_factors_.at(" << i << ")->print(): " << std::endl;
//         //   new_factors_.at(i)->print();

//         //   new_factors_.at(i)->printKeys();

//         //   std::cout << "- Dim: " << new_factors_.at(i)->dim() << std::endl;
//         //   // std::cout << "- Key: " << new_factors_.at(i)->key() << std::endl;
//         //   // std::cout << "- Key1: " << new_factors_.at(i)->key1() << std::endl;
//         //   std::cout << "- Num Keys: " << new_factors_.at(i)->keys().size() << std::endl;
//         //   std::cout << "- Size Keys: " << new_factors_.at(i)->size() << std::endl;

//         //   for ( int j = 0; j < new_factors_.at(i)->keys().size(); j++ )
//         //     std::cout << "\t- Key[" << j << "]: " << new_factors_.at(i)->keys()[j] << std::endl;
//         // }


//         // tryEraseValue(gtsam::symbol_shorthand::V(newest_pose_id));
//         // tryEraseValue(gtsam::symbol_shorthand::B(newest_pose_id));
//         // new_factors_.erase(new_factors_.end());


//         // isam2_ = isam2_error_prone;

//         // isam2_.update(new_factors_, new_values_);
//         // for  (int i = 0; i < num_opt_; ++i)
//         //   isam2_.update(); 


//         // try
//         // {
//         // isam2_.update(new_factors_, new_values_);
//         // for  (int i = 0; i < num_opt_; ++i)
//         //   isam2_.update(); 

//         // }
//         // catch(gtsam::IndeterminantLinearSystemException e){
//         //   exists = new_values_.exists(e.nearbyVariable());
//         //   std::cout << "Exists: " << exists << std::endl;
//         // }
//         // std::cerr << "Optimized 2nd time" << std::endl;
        

//       gtsam::Pose3 pose_pre_optimize = pose_;

//       ROS_INFO_STREAM("ISAM SIZE: "<< isam2_.size() << "Get estimate");

//       // Get updated values
//       gtsam::Values result = isam2_.calculateBestEstimate();
//       tryGetEstimate(gtsam::symbol_shorthand::X(newest_pose_id), result, pose_);

//       if ( ((pose_.translation() - pose_pre_optimize.translation()).norm() > 5) && (isam2_.size() > 5) )
//       {
//         ROS_INFO("ERROR motion");

//         pose_ = pose_pre_optimize;
//         remove_IMUfactor = true;
//         continue;
//       }

//       tryGetEstimate(gtsam::symbol_shorthand::V(newest_pose_id), result, velocity_);
//       tryGetEstimate(gtsam::symbol_shorthand::B(newest_pose_id), result, bias_);
//       current_estimate_ = result;

//       ROS_INFO("GOTTEN");

//       valid_motion = true;
//     }
//     ROS_INFO("OPTIMIZED");


//     // // Get updated values
//     // current_estimate_ = isam2_.calculateBestEstimate();
//     // tryGetEstimate(gtsam::symbol_shorthand::X(newest_pose_id), pose_);
//     // tryGetEstimate(gtsam::symbol_shorthand::V(newest_pose_id), velocity_);
//     // tryGetEstimate(gtsam::symbol_shorthand::B(newest_pose_id), bias_);
    
//     // ROS_INFO("Gotten");

//     // pose_ = current_estimate_.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(newest_pose_id)); // Pose

//     // if ( current_estimate_.exists(gtsam::symbol_shorthand::V(newest_pose_id)) )
//     // {
//     //   if ( ( newest_pose_id < 5 ) || ( ( current_estimate_.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(newest_pose_id)) - velocity_ ).norm() < 2 ) )
//     //     velocity_ = current_estimate_.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(newest_pose_id));  // Velocity
//     // }
//     // if ( current_estimate_.exists(gtsam::symbol_shorthand::B(newest_pose_id)) )
//     //   bias_ = current_estimate_.at<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(newest_pose_id)); // Bias

//     // std::cout << "\nVelocity: " << velocity_.transpose() << std::endl;
//     // pose_.print("--- Optimized Pose ---");

//     // pose_.print("BACKEND OPTIMIZED");
//     world_pose_pub_.publish(generateMsg()); // TODO: Use latest stamp from map


//     // Reset parameters
//     new_factors_.resize(0);
//     new_values_.clear();
//     clearOldAssocations();
//     // imu_.clearOldMeasurements(); 

//     ros::Time toc = ros::Time::now();
//     printSummary((toc - tic).toSec(),
//                   Eigen::Affine3d{pose_.matrix()} );
//   }
// }


geometry_msgs::PoseStamped Backend::generateMsg()
{
  tf2::Stamped<Eigen::Affine3d> tf2_stamped_T(
    Eigen::Isometry3d{pose_.matrix()}, 
    ros::Time::now(), 
    "backend_pose_world"
  );
  geometry_msgs::PoseStamped stamped_pose_msg = tf2::toMsg(tf2_stamped_T);

  return stamped_pose_msg;
}


gtsam::Pose3 Backend::getPoseAt(gtsam::Key key)
{
  if (current_estimate_.exists(key))
    return current_estimate_.at<gtsam::Pose3>(key);
  if (new_values_.exists(key))
    return new_values_.at<gtsam::Pose3>(key);
  else
    return pose_;
}


gtsam::Point3 Backend::getPointAt(gtsam::Key key)
{
  if (current_estimate_.exists(key))
    return current_estimate_.at<gtsam::Point3>(key);
  else if (new_values_.exists(key))
    return new_values_.at<gtsam::Point3>(key);
  else
    return gtsam::Point3();
}


bool Backend::valueExist(gtsam::Key key)
{
  return ( current_estimate_.exists(key) || new_values_.exists(key) );
}


template <typename Value>
bool Backend::tryInsertValue(gtsam::Key key, Value value)
{
  if (! valueExist(key)) // Value doesn't exist, thus is inserted
  {
    new_values_.insert(key, value); 
    return true;
  }
  else
    return false;
}


template <typename Value>
void Backend::forceInsertValue(gtsam::Key key, Value value)
{
  // new_values_.print();
  if (new_values_.exists(key)) // Value exist, thus is deleted
  {
    new_values_.erase(key);
    new_values_.insert(key, value); 
  }
  else
    new_values_.insert(key, value); 
}


bool Backend::tryEraseValue(gtsam::Key key)
{
  if ( new_values_.exists(key) )
  {
    new_values_.erase(key); 
    return true;
  }
  
  return false;
}


template <typename Value>
bool Backend::tryGetEstimate(gtsam::Key key, gtsam::Values result, Value& estimate)
{
  if ( result.exists(key) )
  {
    estimate = result.at<Value>( key ); 
    return true;
  }
  
  return false; 
}


std::pair<int, bool> Backend::searchAssociatedPose(ros::Time pose_stamp, ros::Time prev_pose_stamp)
{    
  std::map<ros::Time, int>::iterator stamped_pose_id = stamped_pose_ids_.begin();
  std::map<ros::Time, int>::iterator prev = stamped_pose_ids_.begin();

  while (stamped_pose_id != stamped_pose_ids_.end())
  {
    if (pose_stamp < stamped_pose_id->first)
      break;

    prev = stamped_pose_id++;
  }

  double prev_diff = std::abs((pose_stamp - prev->first).toSec());
  double diff = std::abs((pose_stamp - stamped_pose_id->first).toSec());

  // Check if associated relative pose. If associated, check if previous or next is closest
  if ( updated_
    && ((prev_diff < association_threshold_) 
    || (diff < association_threshold_)) ) 
  {
    if ( (diff > prev_diff) && (prev->first > prev_pose_stamp)) // Ensure that previous association isn't the actual previous pose 
    {
      stamped_pose_ids_[pose_stamp] = prev->second; 
      return std::make_pair(prev->second, true);            // Prev is associated
    }

    if (stamped_pose_id->first > prev_pose_stamp)
    {
      stamped_pose_ids_[pose_stamp] = stamped_pose_id->second; 
      return std::make_pair(stamped_pose_id->second, true); // Next is associated
    }
  }

  // Not associated pose --> Increase pose_id. 
  stamped_pose_ids_[pose_stamp] = ++pose_id_; 
  return std::make_pair(pose_id_, false);                   // New pose at end
}


void Backend::clearOldAssocations(double interval)
{
  if (stamped_pose_ids_.empty())
    return;

  std::map<ros::Time, int>::iterator stamped_pose_id = stamped_pose_ids_.begin();
  ros::Time newest_stamp = stamped_pose_ids_.rbegin()->first;

  while (stamped_pose_id != stamped_pose_ids_.end())
  {
    if (stamped_pose_id->first > (newest_stamp - ros::Duration(interval)) )
      break;
    
    stamped_pose_id = stamped_pose_ids_.erase(stamped_pose_id);
  }
}


ros::Time Backend::findPoseStamp(int pose_id)
{
  std::map<ros::Time, int>::iterator stamped_pose_id;
  for (stamped_pose_id = stamped_pose_ids_.begin(); stamped_pose_id != stamped_pose_ids_.end(); stamped_pose_id++)
  {
    if (pose_id == stamped_pose_id->second)
      return stamped_pose_id->first;
  }

  return ros::Time(0, 0);
}


bool Backend::isValidMotion(gtsam::Pose3 pose_relative, double x_thresh, double yaw_thresh, double pitch_thresh, double roll_thresh)
{
  return ( ( (pose_relative.x() >= -0.5)
            || ( (pose_relative.x() >= pose_relative.y())
              && (pose_relative.x() >= pose_relative.z()) ) )
          && (std::abs(pose_relative.rotation().yaw()) < yaw_thresh) 
          && (std::abs(pose_relative.rotation().pitch()) < pitch_thresh)
          && (std::abs(pose_relative.rotation().roll()) < roll_thresh) 
          && (! update_contains_loop_) );
}



} // namespace backend