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

/*** Standard library ***/ 
#include <fstream>
#include <time.h>


// #include "backend/factor_handler/imu.h"


namespace backend
{

class Backend
{
private:
  // Nodes
  ros::NodeHandle nh_;
  ros::Publisher  world_pose_pub_;
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
  
  // Current state
  int pose_id_;
  gtsam::Pose3 pose_;
  gtsam::Vector3 velocity_;
  gtsam::imuBias::ConstantBias bias_;

  // Measurement states
  bool nav_status_;                  // is online
  bool initialized_;                 // is online
  std::pair<bool, bool> imu_status_; // <is online, is updated>



  // IMUHelper imu_;


public:
  Backend() 
  : pose_id_(0)
  , num_opt_(20)
  , time_prev_pose_relative_(ros::Time(0.0))
  , initialized_(false)
  , updated_(false)
  , nav_status_(false)
  , imu_status_(std::make_pair(false, false))
  , association_threshold_(0.05)
  , pose_(gtsam::Pose3::identity())
  , velocity_(gtsam::Vector3(0.0, 0.0, 0.0))
  , graph_filename_("graph.dot")
  , result_path_(ros::package::getPath("backend") + "/../../results/")
  , buffer_size_(1000)
  , optimize_timer_(nh_.createTimer(ros::Duration(0.1), &Backend::callback, this))
  , world_pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("/backend/pose", buffer_size_))
  // , imu_(nh_)
  {
    double lag = 2.0;
   
    isam2_params_.relinearizeThreshold = 0.01;
    isam2_params_.relinearizeSkip = 1;
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
  std::pair<bool, bool> checkIMUStatus()  { return imu_status_; }
  std::map<ros::Time, int> getStampedPoseIDs() { return stamped_pose_ids_; }

  int  incrementPoseID()                                { return ++pose_id_; }
  void registerStampedPose(ros::Time stamp, int id)     { stamped_pose_ids_[stamp] = id; }
  void setInitialized(bool status)                      { initialized_ = status; }
  void isUpdated()                                      { updated_ = true; }
  void updatedPreviousRelativeTimeStamp(ros::Time time) { time_prev_pose_relative_ = time; }
  void updateNavStatus(bool status)                     { nav_status_ = status; }
  void updateIMUOnlineStatus(bool status)               { imu_status_.first = status; }
  void updatePreintegrationStatus(bool status)          { imu_status_.second = status; }
  void updateVelocity(gtsam::Vector3 velocity)          { velocity_ = velocity; }
  void updateBias(gtsam::imuBias::ConstantBias bias)    { bias_ = bias; }

  geometry_msgs::PoseStamped generateMsg();
  void callback(const ros::TimerEvent& event);

  bool valueExist(gtsam::Key pose_key);

  template <typename Value>
  bool tryInsertValue(gtsam::Key pose_key, Value value);

  template <typename Value>
  void forceInsertValue(gtsam::Key pose_key, Value value);

  std::pair<int, bool> searchAssociatedPose(ros::Time pose_stamp, ros::Time prev_pose_stamp = ros::Time(0.0));

  template <typename FactorType>
  void addFactor(FactorType factor) { new_factors_.add(factor); }

  void clearOldAssocations(double interval = 1.0);
  ros::Time findPoseStamp(int pose_id);
};


void Backend::callback(const ros::TimerEvent& event)
{
  if (updated_)
  {
    // std::cout << "-----" << std::endl;
    updated_ = false;
    imu_status_.second = false;

    ros::Time tic = ros::Time::now();
    // ros::Time from_time = findPoseStamp(imu_.from_id);
    // ros::Time to_time = stamped_pose_ids_.rbegin()->first; 

    int newest_pose_id = stamped_pose_ids_.rbegin()->second; // Delay with VO - Rather use newest stamped pose (perhaps gnss)

    // new_values_.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id_)).print("\nVO pose: ");

    // if (! imu_.initialized)
    // {      
    //   new_factors_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(0), imu_.priorImuBias, imu_.biasNoiseModel));
    //   new_values_.insert(gtsam::symbol_shorthand::B(0), imu_.priorImuBias);
      
    //   // Velocity prior - assume stationary
    //   new_factors_.add(gtsam::PriorFactor<gtsam::Vector3>(gtsam::symbol_shorthand::V(0), imu_.prior_velocity, imu_.velocityNoiseModel));
    //   new_values_.insert(gtsam::symbol_shorthand::V(0), imu_.prior_velocity);

    //   imu_.initialized = true;
    // }
    // else
    // {
    //   // if (pose_id_ % 10 == 0)
    //   if (to_time > from_time)
    //   {
    //     std::map<ros::Time, IMUMeasurement>::iterator to_it = imu_.integrateMeasurements(from_time, to_time);
    //     imu_.stamped_measurements.erase(imu_.stamped_measurements.begin(), to_it);

    //     imu_.propState = imu_.preintegrated->predict(imu_.prevState, imu_.prevBias);

    //     //Legger inn initial estimates fordi det maa man ha
    //     // forceInsertValue(gtsam::symbol_shorthand::X(newest_pose_id), imu_.propState.pose());
    //     new_values_.insert(gtsam::symbol_shorthand::V(newest_pose_id), imu_.propState.velocity());
    //     new_values_.insert(gtsam::symbol_shorthand::B(newest_pose_id), imu_.prevBias);

    //     // imu_.propState.pose().print("Pred pose: ");
    //     // std::cout << "VO pose: " << new_values_.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id_)).translation().transpose() << std::endl;
    //     std::cout << "pred pose: " << imu_.propState.pose().translation().transpose() << std::endl;
    //     std::cout << "pred velocity: " << imu_.propState.velocity().transpose() << std::endl;
    //     // ROS_INFO_STREAM("pred bias: \n" << imu_.prevBias );

    //     //Her legger vi inn info fra imu i grafen
    //     gtsam::CombinedImuFactor imu_factor(gtsam::symbol_shorthand::X(imu_.from_id), gtsam::symbol_shorthand::V(imu_.from_id),
    //                                         gtsam::symbol_shorthand::X(newest_pose_id), gtsam::symbol_shorthand::V(newest_pose_id), 
    //                                         gtsam::symbol_shorthand::B(imu_.from_id), gtsam::symbol_shorthand::B(newest_pose_id), 
    //                                         *imu_.preintegrated);

    //     new_factors_.add(imu_factor);
    //   }
    // }

    
    // new_values_.print("----- Values -----");
    // new_factors_.print("----- New factors -----");

    // Optimize   
    // try {
    //   isam2_.update(new_factors_, new_values_);
    // } catch(gtsam::IndeterminantLinearSystemException e){
    //   std::cerr << "\n\n\n\n\nCAPTURED THE EXCEPTION: \n\n\n\n\n" << e.nearbyVariable() << std::endl;
    // }

    
    // ROS_INFO_STREAM("TRY OPTIMIZE for POSE: " << newest_pose_id);

    try
    {
      isam2_.update(new_factors_, new_values_);
        // std::cerr << "UPDATED" << std::endl;
      for (int i = 0; i < num_opt_; ++i)
        isam2_.update(); 
      
      // std::cerr << "ALL OK" << std::endl;

    }
    catch(gtsam::IndeterminantLinearSystemException& e){
      std::cerr << "CAPTURED THE EXCEPTION: " << e.nearbyVariable() << std::endl;

      // bool exists = new_values_.exists(e.nearbyVariable());
      // std::cerr << "Exists: " << exists << std::endl;

      // ROS_INFO_STREAM("size nonlingraph: " << new_factors_.size());

      // new_values_.erase(gtsam::symbol_shorthand::V(newest_pose_id));
      // new_values_.erase(gtsam::symbol_shorthand::B(newest_pose_id));
      // new_factors_.erase(new_factors_.end());

      // gtsam::VariableIndex key2index_map_ = isam2_.getVariableIndex();
      // ROS_INFO_STREAM("size map: " << key2index_map_.size());

      // std::cerr << "Optimizing 2nd time" << std::endl;

      // isam2_.print("--- ISAM2 ---");

      // isam2_.update(new_factors_, new_values_);
      // for (int i = 0; i < num_opt_; ++i)
      //   isam2_.update(); 

      // std::cerr << "Optimized 2nd time" << std::endl;
      
      // virtual ISAM2Result update(
      // const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(),
      // const Values& newTheta = Values(),
      // const FactorIndices& removeFactorIndices = FactorIndices(),
    }

    // ROS_INFO("OPTIMIZED");


    // Get updated values
    current_estimate_ = isam2_.calculateBestEstimate();
    // ROS_INFO("Calculated estimate");
    pose_ = current_estimate_.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(newest_pose_id));          // Pose
    // ROS_INFO("Calculated pose");
    if ( current_estimate_.exists(gtsam::symbol_shorthand::V(newest_pose_id)) )
      velocity_ = current_estimate_.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(newest_pose_id));  // Velocity
    if ( current_estimate_.exists(gtsam::symbol_shorthand::B(newest_pose_id)) )
      bias_ = current_estimate_.at<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(newest_pose_id)); // Bias

    // ROS_INFO("Calculated velocity");

    // try
    // {
    //   current_estimate_ = isam2_.calculateBestEstimate();
    //   pose_ = current_estimate_.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(newest_pose_id));          // Pose
    //   if ( current_estimate_.exists(gtsam::symbol_shorthand::V(newest_pose_id)) )
    //     velocity_ = current_estimate_.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(newest_pose_id));  // Velocity
    // }
    // catch(gtsam::IndeterminantLinearSystemException& e){
    // {
    //   ROS_INFO("IGNORE");
    // }
    

    // ROS_INFO("CALCULATED BEST ESTIMATE");


    // pose_.print("BACKEND OPTIMIZED");
    world_pose_pub_.publish(generateMsg());


    // if (pose_id_ % 10 == 0)
    // if (to_time > from_time)
    // {
    //   // std::cout << "Opt pose: " << current_estimate_.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(newest_pose_id)).translation().transpose() << std::endl;

    //   imu_.from_id = newest_pose_id;
    //   imu_.prevState = gtsam::NavState(current_estimate_.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(newest_pose_id)),
    //                                   current_estimate_.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(newest_pose_id)));
    //   imu_.prevBias = current_estimate_.at<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(newest_pose_id));
    //   imu_.preintegrated->resetIntegrationAndSetBias(imu_.prevBias);

    //   // std::cout << "Opt velocity: " << current_estimate_.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(newest_pose_id)).transpose() << std::endl;
    //   // ROS_INFO_STREAM("Opt velocity: \n" << current_estimate_.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(newest_pose_id)) );
    //   // ROS_INFO_STREAM("Opt bias: \n" << current_estimate_.at<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(newest_pose_id)) );
    // }

    // Reset parameters
    new_factors_.resize(0);
    new_values_.clear();
    clearOldAssocations();
    // imu_.clearOldMeasurements(); 

    ros::Time toc = ros::Time::now();
    // ROS_INFO_STREAM("Time per iteration: " <<  (toc - tic) << "\n");
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
  // return ( isam2_.valueExists(key) || new_values_.exists(key) );
  return ( current_estimate_.exists(key) || new_values_.exists(key) );
}


template <typename Value>
bool Backend::tryInsertValue(gtsam::Key pose_key, Value value)
{
  if (! valueExist(pose_key)) // Value doesn't exist, thus is inserted
  {
    new_values_.insert(pose_key, value); 
    return true;
  }
  else
    return false;
}


template <typename Value>
void Backend::forceInsertValue(gtsam::Key pose_key, Value value)
{
  // new_values_.print();
  if (new_values_.exists(pose_key)) // Value exist, thus is deleted
  {
    new_values_.erase(pose_key);
    new_values_.insert(pose_key, value); 
  }
  else
    new_values_.insert(pose_key, value); 
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



} // namespace backend