#pragma once 

/*** ROS packages ***/
#include <ros/ros.h> 
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

/*** GTSAM packages ***/
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
// #include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include "gtsam/nonlinear/Marginals.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam_unstable/nonlinear/ConcurrentIncrementalFilter.h>
#include <gtsam_unstable/nonlinear/ConcurrentIncrementalSmoother.h>

/*** Local ***/ 
#include "support.h"
#include "inputOutput.h"
// #include "backend/motion_model.h"

/*** Standard library ***/ 
#include <string>
#include <fstream>
#include <time.h>


enum { NONE = 0, REMOVE_IMU = 1, INVALID_MOTION = 2, REMOVE_LANDMARK = 3 };


namespace backend
{

class Backend
{
private:
  // Nodes
  ros::NodeHandle nh_;
  ros::Publisher world_pose_pub_;
  ros::Publisher world_cloud_pub_;
  tf2_ros::StaticTransformBroadcaster odom_origin_tf_;
  tf2_ros::TransformBroadcaster odom_world_tf_;

  ros::Timer optimize_timer_;       // Spun node fpr callback
  const int buffer_size_;

  // File
  std::string result_path_;
  std::string filter_filename_;
  std::string smoother_filename_;

  // ISAM2
  int num_opt_;
  gtsam::ISAM2 isam2_; 
  gtsam::ISAM2Params isam2_params_; 
  gtsam::Values current_estimate_; 
  gtsam::Values smoother_estimate_; 

  // Graph 
  bool updated_; 
  gtsam::Values new_values_; 
  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values smoother_values_;
  gtsam::NonlinearFactorGraph smoother_factors_;
  std::map< double, std::vector<gtsam::Key> > landmarks_in_frame_;
  std::vector<gtsam::Key> pending_landmarks_to_remove_;
  std::vector<gtsam::Symbol> removed_symbols_;
  std::vector<gtsam::Symbol> removed_keys_;

  double smooth_lag_;
  double sync_lag_;
  ros::Time prev_sync_;
  ros::Time loop_stamp_;
  std::map<ros::Time, std::vector<gtsam::Key>> stamped_keys_;
  gtsam::ConcurrentIncrementalFilter concurrent_filter_;
  gtsam::ConcurrentIncrementalSmoother concurrent_smoother_;

  // Association
  ros::Time time_prev_pose_relative_;
  double association_threshold_; // in seconds
  std::map<ros::Time, int> stamped_pose_ids_;
  bool update_contains_loop_;
  int loop_id_;
  ros::Time pending_odometry_stamp_;

  // Current state
  int pose_id_;
  gtsam::Pose3 pose_;
  gtsam::Vector3 velocity_;
  gtsam::imuBias::ConstantBias bias_;

  // Measurement states
  bool nav_status_;  // is online
  bool initialized_; // initialized pos by nav

  // World origin
  gtsam::Pose3 world_origin_;

public:
  Backend(boost::property_tree::ptree parameters) 
  : pose_id_(0)
  , num_opt_( parameters.get< int >("number_of_optimizations", 10) )
  , time_prev_pose_relative_(ros::Time(0.0))
  , initialized_(false)
  , updated_(false)
  , nav_status_(false)
  , association_threshold_(0.05)
  , pose_(gtsam::Pose3::identity())
  , velocity_(gtsam::Vector3(0.0, 0.0, 0.0))
  , filter_filename_("filter.dot")
  , smoother_filename_("smoother.dot")
  , result_path_(ros::package::getPath("backend") + "/../../results/")
  , buffer_size_(1000)
  , optimize_timer_(nh_.createTimer(ros::Duration(0.01), &Backend::callback, this))
  , world_pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("/backend/pose", buffer_size_))
  , world_cloud_pub_(nh_.advertise<sensor_msgs::PointCloud2>("/backend/cloud", buffer_size_))
  , update_contains_loop_(false)
  , world_origin_(gtsam::Pose3::identity())
  , pending_odometry_stamp_(ros::Time(0.0))
  , smooth_lag_( parameters.get< double >("smooth_lag", 2.0) )
  , sync_lag_( parameters.get< double >("synchronization_lag", 1.0) )
  , loop_id_(-1)
  {
    isam2_params_.relinearizeThreshold = parameters.get< double >("relinearization_threshold", 0.1);
    isam2_params_.relinearizeSkip = 10;
    isam2_params_.enableRelinearization = true; 
    isam2_params_.evaluateNonlinearError = false; 
    isam2_params_.factorization = gtsam::ISAM2Params::CHOLESKY; 
    isam2_params_.cacheLinearizedFactors = true;
    
    isam2_ = gtsam::ISAM2(isam2_params_);
  };

  ~Backend() 
  {
    concurrent_filter_.getFactors().saveGraph(result_path_ + filter_filename_);
    concurrent_filter_.getISAM2().saveGraph(result_path_ + "isam2_" + filter_filename_);
    concurrent_smoother_.getFactors().saveGraph(result_path_ + smoother_filename_);
  }


  int getPoseID() const                   { return pose_id_; }
  int getNewestPoseID() const             { return stamped_pose_ids_.rbegin()->second; }
  int getSecondNewestPoseID() const;
  ros::Time getNewestPoseTime() const     { return stamped_pose_ids_.rbegin()->first; }
  ros::Time getOldestEstistingPoseTime() const     { return stamped_pose_ids_.begin()->first; }
  gtsam::Values& getValues()              { return new_values_;  }
  gtsam::NonlinearFactorGraph& getGraph() { return new_factors_; }
  gtsam::Pose3 getPose()                  { return pose_; }
  gtsam::Pose3 getPoseAt(gtsam::Key key);
  gtsam::Pose3 getPoseAt(gtsam::Key key, gtsam::Values result);
  gtsam::Point3 getPointAt(gtsam::Key key);
  gtsam::Point3 getPointAt(gtsam::Key key, gtsam::Values result);
  gtsam::Vector3 getVelocity()            { return velocity_; }
  gtsam::imuBias::ConstantBias getBias()  { return bias_; }
  gtsam::ISAM2& getiSAM2()                { return isam2_; }
  bool checkInitialized()                 { return initialized_; }
  bool checkNavStatus()                   { return nav_status_; }
  std::map<ros::Time, int> getStampedPoseIDs() { return stamped_pose_ids_; }
  bool isOdometryPending()                { return (pending_odometry_stamp_ != ros::Time(0.0)); }
  ros::Time newestOdometryStamp()         { return pending_odometry_stamp_; }
  int getPoseIDAtStamp(ros::Time stamp)   { return stamped_pose_ids_[stamp]; }
  bool loopIsAdded()                      { return update_contains_loop_; }

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
  void setWorldOrigin(gtsam::Pose3 pose)                { world_origin_= pose; }
  void setOdometryStamp(ros::Time stamp)                { pending_odometry_stamp_ = stamp; }
  void relateLandmarkToFrame(double frame_stamp, gtsam::Key landmark_key) { landmarks_in_frame_[frame_stamp].push_back(landmark_key); }
  void setLoopStamp(ros::Time stamp)                    { loop_stamp_ = stamp; }
  void setLoopID(int id)                                { loop_id_ = id; }
  void updateSyncStamp(ros::Time stamp)                 { prev_sync_ = stamp; }

  void callback(const ros::TimerEvent& event);

  bool valueExist(gtsam::Key key);
  
  bool valueExistSmoother(gtsam::Key key);

  template <typename Value>
  bool tryInsertValue(gtsam::Key key, Value value);

  template <typename Value>
  bool tryInsertValue(gtsam::Key key, Value value, ros::Time stamp);

  template <typename Value>
  void forceInsertValue(gtsam::Key key, Value value);

  bool tryEraseValue(gtsam::Key key);

  template <typename Value>
  bool tryGetEstimate(gtsam::Key key, gtsam::Values result, Value& estimate);

  template <typename FactorType>
  void addFactor(FactorType factor) { new_factors_.add(factor); }

  template <typename FactorType>
  void addSmootherFactor(FactorType factor) { smoother_factors_.add(factor); }

  std::pair<gtsam::FastList<gtsam::Key>, gtsam::FactorIndices> popKeysOlderThanLag(double lag, std::map<ros::Time, std::vector<gtsam::Key>>& stamped_keys);
  std::pair<gtsam::FastList<gtsam::Key>, gtsam::FactorIndices> popAllShortTermFactors(std::map<ros::Time, std::vector<gtsam::Key>>& stamped_keys);
  gtsam::FactorIndices eraseAllShortTermNonPoseFactors();
  void eraseAllShortTermNonPoseFactors(gtsam::FactorIndices& remove_indices);
  gtsam::FactorIndices eraseAllShortTermIMUFactors();
  void eraseAllShortTermIMUFactors(gtsam::FactorIndices& remove_indices);
  gtsam::FactorIndices eraseShortTermLandmarkFactor(gtsam::Key key);
  void eraseShortTermLandmarkFactor(gtsam::Key key, gtsam::FactorIndices& remove_indices);
  
  void eraseAllNewNonPoseFactors();
  void eraseNewIMUFactors();
  void eraseNewLandmarkFactor(gtsam::Key key);

  std::pair<int, bool> searchAssociatedPose(ros::Time pose_stamp, ros::Time prev_pose_stamp = ros::Time(0.0));

  void clearOldAssocations(double interval = 1.0);
  ros::Time findPoseStamp(int pose_id);
  bool isValidMotion(gtsam::Pose3 pose_relative, bool update_contains_loop, double x_thresh=-0.5, double yaw_thresh=M_PI/9, double pitch_thresh=M_PI/18, double roll_thresh=M_PI/18);
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

    std::pair<gtsam::FastList<gtsam::Key>, gtsam::FactorIndices> popped_variables;
    if (update_contains_loop_)
      popped_variables = popAllShortTermFactors(stamped_keys_);
    else
      popped_variables = popKeysOlderThanLag(smooth_lag_, stamped_keys_);

    gtsam::FastList<gtsam::Key> old_keys = popped_variables.first;
    gtsam::FactorIndices remove_indices = popped_variables.second;

    // Optimize   
    bool invalid_optimization = true;
    int error_correction = NONE;
    gtsam::ISAM2 isam2_error_prone = isam2_; // Save before optimize graph
    gtsam::Values result;
    unsigned char indeterminant_char;
    unsigned int indeterminant_idx;
    bool added_loop_constraint = false;
    bool invalid_motion = false;
    gtsam::FactorIndices error_remove_indices;

    // Save before optimize graph
    gtsam::ConcurrentIncrementalFilter filter_error_prone = concurrent_filter_;
    gtsam::ConcurrentIncrementalSmoother smoother_error_prone = concurrent_smoother_;

    int num_tries = -1;
    while (invalid_optimization)
    {      
      bool value_exist = true;

      if (num_tries++ > 50)
        assert(false);

      try
      {
        while (value_exist)
        {
          try
          {
            concurrent_filter_.update(gtsam::NonlinearFactorGraph(), gtsam::Values(), gtsam::FastList<gtsam::Key>(), error_remove_indices);
            concurrent_filter_.update(new_factors_, new_values_);
            value_exist = false;
          }
          catch(const gtsam::ValuesKeyAlreadyExists e)
          {
            gtsam::Symbol s(e.key());
            std::cerr << "\nValue key " << s.chr() << "(" << s.index() << ")" << " already exist in new values" << std::endl;

            if (new_values_.exists(e.key()))
              new_values_.erase(e.key());

            concurrent_filter_ = filter_error_prone;
          }
        }
        
        // concurrent_filter_.update(new_factors_, new_values_);
        concurrent_filter_.update(gtsam::NonlinearFactorGraph(), gtsam::Values(), gtsam::FastList<gtsam::Key>(), remove_indices);
        concurrent_filter_.update(gtsam::NonlinearFactorGraph(), gtsam::Values(), old_keys, gtsam::FactorIndices());
        for (int i = 0; i < num_opt_; ++i)
          concurrent_filter_.update();


        if ( (prev_sync_ < (getNewestPoseTime() - ros::Duration(sync_lag_))) || update_contains_loop_ )
        {          
          // Synchronize the Filter and Smoother
          concurrent_smoother_.update();
          synchronize(concurrent_filter_, concurrent_smoother_);

          smoother_estimate_ = concurrent_smoother_.calculateEstimate();
          updateSyncStamp( getNewestPoseTime() );
          
          // std::cout << "Smoother size: " << smoother_estimate_.size() << std::endl;
        }

        // if (update_contains_loop_ && smoother_estimate_.exists(gtsam::symbol_shorthand::X(loop_id_)))
        if (update_contains_loop_)
        {
          gtsam::KeyVector loop_keys = smoother_factors_.at(0)->keys();
          gtsam::Symbol s0 = gtsam::Symbol(loop_keys[0]);
          gtsam::Symbol s1 = gtsam::Symbol(loop_keys[1]);
          std::cout << "\n-----\nAdding loop closure between poses <" << s0.chr() << "(" << s0.index() << "), " << s1.chr() << "(" << s1.index() << ")>\n-----" << std::endl;

          concurrent_smoother_.update(smoother_factors_, gtsam::Values());
          for (int i = 0; i < num_opt_; i++)
            concurrent_smoother_.update();

          synchronize(concurrent_filter_, concurrent_smoother_);
          concurrent_filter_.update();

          added_loop_constraint = true;
          smoother_estimate_ = concurrent_smoother_.calculateEstimate();
        }


        result = concurrent_filter_.calculateEstimate();
        // std::cout << "Filter updated size: " << result.size() << std::endl;
        gtsam::Pose3 pose_previous = pose_;
        gtsam::Pose3 pose_current;
        tryGetEstimate(gtsam::symbol_shorthand::X(newest_pose_id), result, pose_current);

        // Ensure valid motion if all conditions are met
        gtsam::Pose3 pose_relative = pose_previous.between(pose_current);

        // error_correction = NONE;

        // if (! isValidMotion(pose_relative, update_contains_loop_, -0.5, M_PI / 9, M_PI / 18, M_PI / 18) )
        //   std::cout << "\n\n\n\n\n\n\n--------------------------------------------" << std::endl;

        if ( isValidMotion(pose_relative, update_contains_loop_, -0.5, M_PI / 9, M_PI / 18, M_PI / 18) )
          error_correction = NONE;
        else
          error_correction = (error_correction == INVALID_MOTION ? NONE : INVALID_MOTION );
      }
      catch(gtsam::IndeterminantLinearSystemException e)
      {
        gtsam::Symbol symb(e.nearbyVariable());
        indeterminant_char = symb.chr();
        indeterminant_idx = symb.index();
        std::cerr << "\nIndeterminant Linear System for " << indeterminant_char << "(" << indeterminant_idx << ")" << std::endl;

        if (error_correction == INVALID_MOTION)
          error_correction = NONE; 
        else
        { 
          if ( (indeterminant_char == 'v') || (indeterminant_char == 'b') )
            error_correction = (error_correction == REMOVE_IMU ? NONE : REMOVE_IMU );
          else if (indeterminant_char == 'l') 
            error_correction = REMOVE_LANDMARK;
          else
            error_correction = (error_correction == REMOVE_IMU ? INVALID_MOTION : REMOVE_IMU ); 
        }
      }


      switch (error_correction)
      {
      case REMOVE_IMU:
      {
        std::cout << "REMOVE_IMU" << std::endl;

        concurrent_filter_ = filter_error_prone;
        concurrent_smoother_ = smoother_error_prone;
        
        // Typically the IMU is the issue --> remove IMUfactors
        eraseNewIMUFactors();
        eraseAllShortTermIMUFactors(error_remove_indices);
        
        // gtsam::FactorIndices error_remove_indices = eraseAllShortTermIMUFactors();
        // concurrent_filter_.update(gtsam::NonlinearFactorGraph(), gtsam::Values(), gtsam::FastList<gtsam::Key>(), error_remove_indices);
        // filter_error_prone = concurrent_filter_;

        invalid_optimization = true;
        break;
      }

      case INVALID_MOTION:
      {
        std::cout << "INVALID_MOTION" << std::endl;

        concurrent_filter_ = filter_error_prone;
        concurrent_smoother_ = smoother_error_prone;
        
        // Remove both new landmarks and IMU factors
        eraseAllNewNonPoseFactors();
        eraseAllShortTermNonPoseFactors(error_remove_indices);

        // gtsam::FactorIndices error_remove_indices = eraseAllShortTermNonPoseFactors();
        // concurrent_filter_.update(gtsam::NonlinearFactorGraph(), gtsam::Values(), gtsam::FastList<gtsam::Key>(), error_remove_indices);
        // filter_error_prone = concurrent_filter_;

        invalid_optimization = true;
        break;
      }

      case REMOVE_LANDMARK:
      {
        std::cout << "REMOVE_LANDMARK" << std::endl;

        concurrent_filter_ = filter_error_prone;
        concurrent_smoother_ = smoother_error_prone;
        
        gtsam::Symbol key(indeterminant_char, indeterminant_idx);

        if ( (current_estimate_.exists(key)) && (std::find(removed_keys_.begin(), removed_keys_.end(), key) != removed_keys_.end()) ) // If landmark is already removed from new factors
        {     
          // gtsam::FactorIndices error_remove_indices = eraseShortTermLandmarkFactor(gtsam::Key(key));
          // concurrent_filter_.update(gtsam::NonlinearFactorGraph(), gtsam::Values(), gtsam::FastList<gtsam::Key>(), error_remove_indices);
          // filter_error_prone = concurrent_filter_;

          eraseShortTermLandmarkFactor(gtsam::Key(key), error_remove_indices);
        }
        else // If landmark exist in new factors it is clearly unstable --> remove it
        {
          eraseNewLandmarkFactor(key);
          removed_keys_.push_back(key);
        }

        invalid_optimization = true;
        break;
      }

      case NONE:
      default:
        tryGetEstimate(gtsam::symbol_shorthand::X(newest_pose_id), result, pose_);
        tryGetEstimate(gtsam::symbol_shorthand::V(newest_pose_id), result, velocity_);
        tryGetEstimate(gtsam::symbol_shorthand::B(newest_pose_id), result, bias_);
        current_estimate_ = concurrent_filter_.calculateEstimate();
        invalid_optimization = false;
        break;
      }
    }

    // End of iteration updates
    odom_origin_tf_.sendTransform( generateOdomOriginMsg(getNewestPoseTime(), world_origin_, "odom") );
    // odom_world_tf_.sendTransform( generateOdomOriginMsg(ros::Time::now(), pose_, "body") );
    
    world_pose_pub_.publish( generatePoseMsg(getNewestPoseTime(), pose_) ); // TODO: Use latest stamp from map
    // world_cloud_pub_.publish( generateCloudMsg(ros::Time::now(), current_estimate_) );

    // Reset parameters
    new_factors_.resize(0);
    new_values_.clear();
    clearOldAssocations();
    removed_keys_.clear();
    
    if (added_loop_constraint)
    {
      update_contains_loop_ = false;
      smoother_factors_.resize(0);
    }
    ros::Time toc = ros::Time::now();
    printSummary((toc - tic).toSec(),
                  Eigen::Affine3d{pose_.matrix()} );
  }
}


int Backend::getSecondNewestPoseID() const
{ 
  // std::map<ros::Time, int>::reverse_iterator stamped_id = stamped_pose_ids_.rbegin();
  auto stamped_id = stamped_pose_ids_.rbegin();

  gtsam::Symbol s;
  if (stamped_pose_ids_.size() >= 2)
  {
    stamped_id++;
    s = gtsam::Symbol(stamped_id->second);
  }
  else
    s = gtsam::Symbol(stamped_id->second);

  return s.index();
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


gtsam::Pose3 Backend::getPoseAt(gtsam::Key key, gtsam::Values result)
{
  if (result.exists(key))
    return result.at<gtsam::Pose3>(key);
  if (new_values_.exists(key))
    return new_values_.at<gtsam::Pose3>(key);
  else
    return gtsam::Pose3();
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

gtsam::Point3 Backend::getPointAt(gtsam::Key key, gtsam::Values result)
{
  if (result.exists(key))
    return result.at<gtsam::Point3>(key);
  else if (new_values_.exists(key))
    return new_values_.at<gtsam::Point3>(key);
  else
    return gtsam::Point3();
}

bool Backend::valueExist(gtsam::Key key)
{
  return ( current_estimate_.exists(key) || new_values_.exists(key) );
}


bool Backend::valueExistSmoother(gtsam::Key key)
{
  return ( current_estimate_.exists(key) || smoother_values_.exists(key) );
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
bool Backend::tryInsertValue(gtsam::Key key, Value value, ros::Time stamp)
{
  if (! valueExist(key)) // Value doesn't exist, thus is inserted
  {
    stamped_keys_[stamp].push_back(key);
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


std::pair<gtsam::FastList<gtsam::Key>, gtsam::FactorIndices> Backend::popKeysOlderThanLag(double lag, std::map<ros::Time, std::vector<gtsam::Key>>& stamped_keys)
{
  gtsam::FactorIndices remove_indices;
  gtsam::FastList<gtsam::Key> old_keys;
  removed_symbols_.clear();

  if (stamped_keys.empty())
    return std::make_pair(old_keys, remove_indices);

  gtsam::VariableIndex key2factor_indices = concurrent_filter_.getISAM2().getVariableIndex();
  gtsam::NonlinearFactorGraph filter_fg = concurrent_filter_.getFactors();
  // gtsam::Values filter_result = concurrent_filter_.calculateEstimate();
  
  std::map<ros::Time, std::vector<gtsam::Key>>::iterator stamped_key = stamped_keys.begin();
  ros::Time newest_stamp = stamped_keys.rbegin()->first;

  while (stamped_key != stamped_keys.end())
  {
    if (stamped_key->first > (newest_stamp - ros::Duration(lag)) )
      break;
    
    std::vector<gtsam::Key> keys = stamped_key->second;
    
    // std::cout << "popKeysOlderThanLag() - keys.size: " << keys.size() << std::endl;

    for(gtsam::Key key : keys)
    {
      gtsam::Symbol symb(key);
      try
      {
        gtsam::FactorIndices factor_indices = key2factor_indices[key];
        // std::cout << "poplag(): " <<  symb.chr() << "(" << symb.index() << ") - number of factor indices: " << factor_indices.size() << std::endl;
        
        if (symb.chr() == 'x')
        {
          old_keys.push_back(key);
          int j = 0;
          for(gtsam::FactorIndex index : factor_indices)
          {
            if (std::string(typeid(*filter_fg[index]).name()).substr(9,19) == "GenericStereoFactor")
              remove_indices.push_back(index);
  
            if (std::string(typeid(*filter_fg[index]).name()).substr(9,17) == "CombinedImuFactor") 
              remove_indices.push_back(index);

          }
        }
        else if (symb.chr() == 'l')
        {
          for(gtsam::FactorIndex index : factor_indices)
          {
            if (std::string(typeid(*filter_fg[index]).name()).substr(9,11) == "PriorFactor")
              remove_indices.push_back(index);
          }
        }
        else if ( (symb.chr() == 'v') || (symb.chr() == 'b'))
        {
          for(gtsam::FactorIndex index : factor_indices)
          {
            if (std::string(typeid(*filter_fg[index]).name()).substr(9,11) == "PriorFactor")
              remove_indices.push_back(index);
          }
        }
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
    }
    // old_keys.push_back(stamped_key->second);
    stamped_key = stamped_keys.erase(stamped_key);
  }

  return std::make_pair(old_keys, remove_indices);
}


std::pair<gtsam::FastList<gtsam::Key>, gtsam::FactorIndices> Backend::popAllShortTermFactors(std::map<ros::Time, std::vector<gtsam::Key>>& stamped_keys)
{
  gtsam::FactorIndices remove_indices;
  gtsam::FastList<gtsam::Key> old_keys;

  gtsam::VariableIndex key2factor_indices = concurrent_filter_.getISAM2().getVariableIndex();
  gtsam::NonlinearFactorGraph filter_fg = concurrent_filter_.getFactors();
  gtsam::Values filter_estimate = concurrent_filter_.calculateEstimate();
  
  for (gtsam::Key key : filter_estimate.keys())
  {
    gtsam::Symbol symb(key);
    
    if (symb.chr() == 'x')
        old_keys.push_back(key);

    try
    {
      gtsam::FactorIndices factor_indices = key2factor_indices[key];
      // for(gtsam::FactorIndex index : factor_indices)
      // {        
      //   if (std::string(typeid(*filter_fg[index]).name()).substr(9,13) != "BetweenFactor")
      //     remove_indices.push_back(index);
      // }
      for(gtsam::FactorIndex index : factor_indices)
      {
        if ( (std::string(typeid(*filter_fg[index]).name()).substr(9,19) == "GenericStereoFactor") 
          || (std::string(typeid(*filter_fg[index]).name()).substr(9,11) == "PriorFactor") 
          || (std::string(typeid(*filter_fg[index]).name()).substr(9,17) == "CombinedImuFactor") )
        {
          remove_indices.push_back(index);
        }
      }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    
  }

  gtsam::KeyVector new_keys = new_values_.keys();

  std::map<ros::Time, std::vector<gtsam::Key>>::iterator stamped_key = stamped_keys.begin();

  while (stamped_key != stamped_keys.end())
  { 
    bool remove = true;
    std::vector<gtsam::Key> keys = stamped_key->second;
    for(gtsam::Key key : keys)
    {    
      if ( std::find(new_keys.begin(), new_keys.end(), key) != new_keys.end() )
        remove = false;
    }
    if (remove)
      stamped_key = stamped_keys.erase(stamped_key);
    else
      stamped_key++;
  }

  return std::make_pair(old_keys, remove_indices);
}


gtsam::FactorIndices Backend::eraseAllShortTermIMUFactors()
{
  gtsam::FactorIndices remove_indices;

  gtsam::VariableIndex key2factor_indices = concurrent_filter_.getISAM2().getVariableIndex();
  gtsam::NonlinearFactorGraph filter_fg = concurrent_filter_.getFactors();
  gtsam::Values filter_estimate = concurrent_filter_.calculateEstimate();
  
  for (gtsam::Key key : filter_estimate.keys())
  {
    gtsam::Symbol symb(key);
    if (symb.chr() == 'v')    
    {
      try
      {
        gtsam::FactorIndices factor_indices = key2factor_indices[key];
        for(gtsam::FactorIndex index : factor_indices)
        {
          if (std::string(typeid(*filter_fg[index]).name()).substr(9,17) == "CombinedImuFactor") 
            remove_indices.push_back(index);
        }
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
      
    }
  }

  return remove_indices;
}


void Backend::eraseAllShortTermIMUFactors(gtsam::FactorIndices& remove_indices)
{
  gtsam::VariableIndex key2factor_indices = concurrent_filter_.getISAM2().getVariableIndex();
  gtsam::NonlinearFactorGraph filter_fg = concurrent_filter_.getFactors();
  gtsam::Values filter_estimate = concurrent_filter_.calculateEstimate();
  
  for (gtsam::Key key : filter_estimate.keys())
  {
    gtsam::Symbol symb(key);
    if (symb.chr() == 'v')    
    {
      try
      {
        gtsam::FactorIndices factor_indices = key2factor_indices[key];
        for(gtsam::FactorIndex index : factor_indices)
        {
          if (std::string(typeid(*filter_fg[index]).name()).substr(9,17) == "CombinedImuFactor") 
            remove_indices.push_back(index);
        }
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
      
    }
  }
}


gtsam::FactorIndices Backend::eraseShortTermLandmarkFactor(gtsam::Key key)
{
  gtsam::FactorIndices remove_indices;

  gtsam::NonlinearFactorGraph filter_fg = concurrent_filter_.getFactors();
  gtsam::VariableIndex key2factor_indices = concurrent_filter_.getISAM2().getVariableIndex();
  gtsam::FactorIndices factor_indices = key2factor_indices[key];

  for(gtsam::FactorIndex index : factor_indices)
    if ( (std::string(typeid(*filter_fg[index]).name()).substr(9,19) == "GenericStereoFactor") 
      || (std::string(typeid(*filter_fg[index]).name()).substr(9,11) == "PriorFactor") )
      remove_indices.push_back(index);

  return remove_indices;
}


void Backend::eraseShortTermLandmarkFactor(gtsam::Key key, gtsam::FactorIndices& remove_indices)
{
  gtsam::NonlinearFactorGraph filter_fg = concurrent_filter_.getFactors();
  gtsam::VariableIndex key2factor_indices = concurrent_filter_.getISAM2().getVariableIndex();
  gtsam::FactorIndices factor_indices = key2factor_indices[key];

  for(gtsam::FactorIndex index : factor_indices)
    if ( (std::string(typeid(*filter_fg[index]).name()).substr(9,19) == "GenericStereoFactor") 
      || (std::string(typeid(*filter_fg[index]).name()).substr(9,11) == "PriorFactor") )
      remove_indices.push_back(index);
}



gtsam::FactorIndices Backend::eraseAllShortTermNonPoseFactors()
{
  gtsam::FactorIndices remove_indices;

  gtsam::VariableIndex key2factor_indices = concurrent_filter_.getISAM2().getVariableIndex();
  gtsam::NonlinearFactorGraph filter_fg = concurrent_filter_.getFactors();
  gtsam::Values filter_estimate = concurrent_filter_.calculateEstimate();
  
  for (gtsam::Key key : filter_estimate.keys())
  {
    gtsam::Symbol symb(key);
    
    if (symb.chr() == 'x')    
    {
      try
      {
        gtsam::FactorIndices factor_indices = key2factor_indices[key];
        for(gtsam::FactorIndex index : factor_indices)
          if (std::string(typeid(*filter_fg[index]).name()).substr(9,13) != "BetweenFactor")
            remove_indices.push_back(index);
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
      
    }
  }

  return remove_indices;
}


void Backend::eraseAllShortTermNonPoseFactors(gtsam::FactorIndices& remove_indices)
{
  gtsam::VariableIndex key2factor_indices = concurrent_filter_.getISAM2().getVariableIndex();
  gtsam::NonlinearFactorGraph filter_fg = concurrent_filter_.getFactors();
  gtsam::Values filter_estimate = concurrent_filter_.calculateEstimate();
  
  for (gtsam::Key key : filter_estimate.keys())
  {
    gtsam::Symbol symb(key);
    
    if (symb.chr() == 'x')    
    {
      try
      {
        gtsam::FactorIndices factor_indices = key2factor_indices[key];
        for(gtsam::FactorIndex index : factor_indices)
          if (std::string(typeid(*filter_fg[index]).name()).substr(9,13) != "BetweenFactor")
            remove_indices.push_back(index);
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
      
    }
  }
}


void Backend::eraseAllNewNonPoseFactors()
{
  for ( int i = 0; i < new_factors_.size(); )
  {
    bool remove = false;
    for ( int j = 0; j < new_factors_.at(i)->size(); j++)
    {
      gtsam::Symbol symb(new_factors_.at(i)->keys()[j]);
      if ( (symb.chr() != 'x') 
        || (std::string(typeid(*new_factors_[i]).name()).substr(9,13) != "BetweenFactor") )
      {
        tryEraseValue(gtsam::Key(symb)); 
        remove = true;
      }
    }

    if (remove)
      new_factors_.erase( new_factors_.begin() + i ); // IMUfactor
    else
      i++;
  }
}

void Backend::eraseNewIMUFactors()
{
  for ( int i = 0; i < new_factors_.size(); )
  {
    // if ( new_factors_.at(i)->size() == 6 )
    if (std::string(typeid(*new_factors_[i]).name()).substr(9,17) == "CombinedImuFactor") 
    {
      tryEraseValue(new_factors_.at(i)->keys()[3]); // Velocity_to key
      tryEraseValue(new_factors_.at(i)->keys()[5]); // Bias_to key

      new_factors_.erase( new_factors_.begin() + i ); // IMUfactor
    }
    else
      i++;
  }
}


void Backend::eraseNewLandmarkFactor(gtsam::Key key)
{
  // Remove initial estimate of measurement
  tryEraseValue(key); // Landmark key

  // Remove projection factor
  for ( int i = 0; i < new_factors_.size(); i++)
  {
    gtsam::KeyVector::const_iterator found_it = new_factors_.at(i)->find( key );

    if ( found_it != new_factors_.at(i)->end() )
      new_factors_.erase( new_factors_.begin() + i ); 
    else
      i++;
  }
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


bool Backend::isValidMotion(gtsam::Pose3 pose_relative, bool update_contains_loop, double x_thresh, double yaw_thresh, double pitch_thresh, double roll_thresh)
{
  return ( ( (pose_relative.x() >= -0.5)
            || ( (pose_relative.x() >= pose_relative.y())
              && (pose_relative.x() >= pose_relative.z()) ) )
          && (std::abs(pose_relative.rotation().yaw()) < yaw_thresh) 
          && (std::abs(pose_relative.rotation().pitch()) < pitch_thresh)
          && (std::abs(pose_relative.rotation().roll()) < roll_thresh)
          || (update_contains_loop) );
}



} // namespace backend