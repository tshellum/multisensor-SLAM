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
#include "inputOutput.h"
// #include "backend/motion_model.h"

/*** Standard library ***/ 
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
  std::map< double, std::vector<gtsam::Key> > landmarks_in_frame_;
  std::vector<gtsam::Key> pending_landmarks_to_remove_;

  // Association
  ros::Time time_prev_pose_relative_;
  double association_threshold_; // in seconds
  std::map<ros::Time, int> stamped_pose_ids_;
  bool update_contains_loop_;
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
  , graph_filename_("graph.dot")
  , result_path_(ros::package::getPath("backend") + "/../../results/")
  , buffer_size_(1000)
  , optimize_timer_(nh_.createTimer(ros::Duration(0.1), &Backend::callback, this))
  , world_pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("/backend/pose", buffer_size_))
  , world_cloud_pub_(nh_.advertise<sensor_msgs::PointCloud2>("/backend/cloud", buffer_size_))
  , update_contains_loop_(false)
  , world_origin_(gtsam::Pose3::identity())
  , pending_odometry_stamp_(ros::Time(0.0))
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
  bool isOdometryPending()                { return (pending_odometry_stamp_ != ros::Time(0.0)); }
  ros::Time newestOdometryStamp()         { return pending_odometry_stamp_; }
  int getPoseIDAtStamp(ros::Time stamp)   { return stamped_pose_ids_[stamp]; }

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

  bool isLeaf(gtsam::Key key, gtsam::ISAM2 graph);
  gtsam::FastList<gtsam::Key> findAllLeafNodeLandmarks(gtsam::ISAM2 graph);
  gtsam::FastList<gtsam::Key> popLeafNodeLandmarksOlderThanLag(double lag, gtsam::ISAM2 graph, std::map< double, std::vector<gtsam::Key> >& landmarks_in_frame, std::vector<gtsam::Key>& pending_landmarks_to_remove);

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

    if ( update_contains_loop_ )
    {  
      std::cout << "\nNumber of values before marginalization: " << current_estimate_.size() << std::endl;
      try 
      {
        isam2_.marginalizeLeaves( findAllLeafNodeLandmarks(isam2_) );
        std::cout << "Marginalized before loop closure added..." << std::endl;
      }
      catch(std::invalid_argument& e)
      {
        std::cout << "Marginalization error" << std::endl;
      }
    }

    // Optimize   
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

        // Remove both landmarks and IMU factors
        for ( int i = 0; i < new_factors_.size(); )
        {
          bool remove = false;
          for ( int j = 0; j < new_factors_.at(i)->size(); j++)
          {
            gtsam::Symbol symb(new_factors_.at(i)->keys()[j]);
            if (symb.chr() != 'x')
            {
              tryEraseValue(new_factors_.at(i)->keys()[j]); 
              remove = true;
            }   
          }

          if (remove)
            new_factors_.erase( new_factors_.begin() + i ); // IMUfactor
          else
            i++;
        }

        invalid_optimization = true;
        break;
      }

      case REMOVE_LANDMARK:
      {
        isam2_ = isam2_error_prone;
        remove_indices = gtsam::FactorIndices();
        gtsam::Symbol key(indeterminant_char, indeterminant_idx);

        if ( current_estimate_.exists(key) && isLeaf(key, isam2_) )
        {
          gtsam::FastList<gtsam::Key> remove_indices;
          remove_indices.push_back(key);
          isam2_.marginalizeLeaves( remove_indices );

          // gtsam::VariableIndex key2factor_index = isam2_.getVariableIndex();
          // remove_indices = key2factor_index[gtsam::Symbol(indeterminant_char, indeterminant_idx)];
        }

        // Remove initial estimate of measurement
        tryEraseValue(key); // Landmark key

        // Remove GenericProjectionFactor
        for ( int i = 0; i < new_factors_.size(); )
        {
          gtsam::KeyVector::const_iterator found_it = new_factors_.at(i)->find( key );

          if ( found_it != new_factors_.at(i)->end() )
          {
            new_factors_.erase( new_factors_.begin() + i ); // Generic projection factor
          }
          else
            i++;
        }

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

    if ( update_contains_loop_ )
      std::cout << "Number of values after marginalization: " << current_estimate_.size() << std::endl;

    // std::cout << "\n" << num_tries << " landmarks removed: " << std::endl;


    // End of iteration updates
    odom_origin_tf_.sendTransform( generateOdomOriginMsg(getNewestPoseTime(), world_origin_, "odom") );
    // odom_world_tf_.sendTransform( generateOdomOriginMsg(ros::Time::now(), pose_, "body") );
    
    world_pose_pub_.publish( generatePoseMsg(getNewestPoseTime(), pose_) ); // TODO: Use latest stamp from map
    // world_cloud_pub_.publish( generateCloudMsg(ros::Time::now(), current_estimate_) );

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


bool Backend::isLeaf(gtsam::Key key, gtsam::ISAM2 graph)
{
  gtsam::BayesTree<gtsam::ISAM2Clique>::Nodes cliques = graph.nodes();
  for (std::pair<gtsam::Key, boost::shared_ptr<gtsam::ISAM2Clique>> clique : cliques)
  {
    if (key == clique.first)
      return true;
  }

  return false;
}


gtsam::FastList<gtsam::Key> Backend::findAllLeafNodeLandmarks(gtsam::ISAM2 graph)
{
  gtsam::FastList<gtsam::Key> variables_to_marginalize;

  gtsam::BayesTree<gtsam::ISAM2Clique>::Nodes cliques = graph.nodes();
  for (std::pair<gtsam::Key, boost::shared_ptr<gtsam::ISAM2Clique>> clique : cliques)
  {
    gtsam::Symbol symb(clique.first);
    if ( (symb.chr() == 'l') && (clique.second->treeSize() == 1) )
      variables_to_marginalize.push_back( gtsam::Key(symb) );
  }

  return variables_to_marginalize;
}


gtsam::FastList<gtsam::Key> Backend::popLeafNodeLandmarksOlderThanLag(double lagged_stamp, gtsam::ISAM2 graph, std::map< double, std::vector<gtsam::Key> >& landmarks_in_frame, std::vector<gtsam::Key>& pending_landmarks_to_remove)
{
  gtsam::FastList<gtsam::Key> variables_to_marginalize;

  // Add old landmarks - should be no duplicates
  std::map<double, std::vector<gtsam::Key>>::iterator stamped_key;
  for (stamped_key = landmarks_in_frame.begin(); stamped_key != landmarks_in_frame.end(); )
  {
    if (stamped_key->first > lagged_stamp)
      break;

    std::vector<gtsam::Key> keys_at_stamp = stamped_key->second;
    pending_landmarks_to_remove.insert(pending_landmarks_to_remove.end(), keys_at_stamp.begin(), keys_at_stamp.end());

    stamped_key = landmarks_in_frame.erase(stamped_key);
  }  

  std::vector<gtsam::Key>::iterator key_it;
  for(key_it = pending_landmarks_to_remove.begin(); key_it != pending_landmarks_to_remove.end(); )
  {
    if ( isLeaf(*key_it, graph) )
    {
      variables_to_marginalize.push_back(*key_it);
      key_it = pending_landmarks_to_remove.erase(key_it);
    }
    else
      key_it++;
  }

  return variables_to_marginalize;
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