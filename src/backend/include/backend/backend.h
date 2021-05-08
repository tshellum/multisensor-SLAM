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


enum { NONE = 0, REMOVE_IMU = 1, INVALID_MOTION = 2, REMOVE_LANDMARK = 3, INDETERMINANT_LINEAR_SYSTEM = 4 };


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
  , num_opt_(10)
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
  ros::Time getOldestEstistingPoseTime() const     { return stamped_pose_ids_.begin()->first; }
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
    unsigned char indeterminant_char;
    unsigned int indeterminant_idx;
    gtsam::FactorIndices remove_indices;

    int num_tries = -1;
    while (invalid_optimization)
    {      
      if (num_tries++ > 500)
        assert(false);

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
          error_correction = (error_correction == REMOVE_IMU ? NONE : REMOVE_IMU );

      }
      catch(gtsam::IndeterminantLinearSystemException e)
      {
        gtsam::Symbol symb(e.nearbyVariable());
        indeterminant_char = symb.chr();
        indeterminant_idx = symb.index();
        std::cerr << "\nIndeterminant Linear System for " << indeterminant_char << "(" << indeterminant_idx << ")" << std::endl;


        if ( (indeterminant_char == 'v') || (indeterminant_char == 'b') )
          error_correction = (error_correction == REMOVE_IMU ? NONE : REMOVE_IMU );
        else if (indeterminant_char == 'l') 
          error_correction = REMOVE_LANDMARK;
        else
          error_correction = (error_correction == REMOVE_IMU ? NONE : REMOVE_IMU ); // TODO: Should be INDETERMINANT_LINEAR_SYSTEM

      }


      switch (error_correction)
      {
      case REMOVE_IMU:
      {
        isam2_ = isam2_error_prone;
        remove_indices = gtsam::FactorIndices();

        // Typically the IMU is the issue --> remove IMUfactors
        for ( int i = 0; i < new_factors_.size(); )
        {
          if ( new_factors_.at(i)->size() == 6 )
          {
            tryEraseValue(new_factors_.at(i)->keys()[3]); // Velocity_to key
            tryEraseValue(new_factors_.at(i)->keys()[5]); // Bias_to key

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
        remove_indices = gtsam::FactorIndices();

        // Fix
        invalid_optimization = true;
        break;
      }
      case REMOVE_LANDMARK:
      {
        isam2_ = isam2_error_prone;
        remove_indices = gtsam::FactorIndices();

        // std::cout << indeterminant_char << "(" << indeterminant_idx << ") exist in new values? : " << new_values_.exists(gtsam::Symbol(indeterminant_char, indeterminant_idx)) << std::endl;
        // std::cout << indeterminant_char << "(" << indeterminant_idx << ") exist in isam2? : " << current_estimate_.exists(gtsam::Symbol(indeterminant_char, indeterminant_idx)) << std::endl;

        tryEraseValue(gtsam::Symbol(indeterminant_char, indeterminant_idx)); // Landmark key

        if ( current_estimate_.exists(gtsam::Symbol(indeterminant_char, indeterminant_idx)) )
        {
          gtsam::VariableIndex key2factor_index = isam2_.getVariableIndex();
          remove_indices = key2factor_index[gtsam::Symbol(indeterminant_char, indeterminant_idx)];

          // std::cout << "Remove indices size: " << remove_indices.size() << std::endl;

          // assert(false);
        }

        // Typically the IMU is the issue --> remove IMUfactors
        for ( int i = 0; i < new_factors_.size(); )
        {
          gtsam::KeyVector::const_iterator found_it = new_factors_.at(i)->find( gtsam::Symbol(indeterminant_char, indeterminant_idx) );

          if ( found_it != new_factors_.at(i)->end() )
          {
            // std::cout << "factor(" << i << ") found" << std::endl;

            new_factors_.erase( new_factors_.begin() + i ); // Generic projection factor
          }
          else
            i++;
        }

        invalid_optimization = true;
        break;
      }
      case INDETERMINANT_LINEAR_SYSTEM:
      {
        isam2_ = isam2_error_prone;
        remove_indices = gtsam::FactorIndices();

        // gtsam::VariableIndex key2factor_index = isam2_.getVariableIndex();
        // remove_indices = key2factor_index[gtsam::Symbol(indeterminant_char, indeterminant_idx)];
        invalid_optimization = true;
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

    // std::cout << "\nNum landmarks removed: " << num_tries << std::endl;

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