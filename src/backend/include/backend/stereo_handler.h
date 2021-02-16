/*** ROS packages ***/
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

/*** GTSAM packages ***/
#include <gtsam/slam/BetweenFactor.h>



class StereoHandler
{
private:
  // double _timestamp;
  // std::vector<double> _timestamps;

  gtsam::Rot3   _rotation;
  gtsam::Point3 _translation;
  gtsam::noiseModel::Diagonal::shared_ptr _NOISE;
public:
  StereoHandler() 
  {
    gtsam::Vector6 sigmas;
    sigmas << gtsam::Vector3(1.0, 1.0, 1.0), gtsam::Vector3(1.0, 1.0, 1.0);
    _NOISE = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  };
  ~StereoHandler() {};

  // int searchCorrespondence(double timestamp, std::vector<double>& pose_timestamps);
  // int searchCorrespondence(double timestamp, std::map<double, int>& timestamped_ids);
  int searchCorrespondence(ros::Time stamp, std::map<double, int>& timestamped_ids);

  void addPose2Graph(int& pose_id, std::map<double, int>& timestamped_ids,
                     const geometry_msgs::PoseStampedConstPtr &pose_msg, gtsam::NonlinearFactorGraph& graph);
  void addCloud2Graph(int pose_id,  
                      const sensor_msgs::PointCloud2ConstPtr &cloud_msg, gtsam::NonlinearFactorGraph& graph);

};


// int StereoHandler::searchCorrespondence(double timestamp, std::vector<double>& pose_timestamps);
// {
//   // Evaluate new measurement timestamp to find where previous measurement from other factors took place
//   int pose_id = my_vector.size()-1;
//   while (pose_timestamps[pose_id] > timestamp)
//     pose_id--;

//   // Future poses are h 
//   pose_timestamps.erase(pose_timestamps.begin(), pose_timestamps.begin() + pose_id);

//   return pose_id;
// }


// int StereoHandler::searchCorrespondence(double timestamp, std::map<double, int>& timestamped_ids)
// {
//   // Create a map iterator and point to the end of map
//   // std::map<double, int>::reverse_iterator stamp = timestamped_ids.rbegin();
//   std::map<double, int>::iterator stamp = timestamped_ids.end();
//   if (!timestamped_ids.empty())
//     stamp--;

//   // Evaluate new measurement timestamp to find where previous measurement from other factors took place

//   ROS_INFO_STREAM("timestamp: " << timestamp);
//   ROS_INFO_STREAM("pose_relative_id->time: " << stamp->first);
//   ROS_INFO_STREAM("pose_relative_id->id: " << stamp->second);
//   bool left_a = timestamp > stamp->first;
//   ROS_INFO_STREAM("timestamp > pose_relative_id->time: " << left_a);
  
//   while (stamp->first > timestamp)
//   {
//     stamp--;
//     left_a = timestamp > stamp->first;
//     ROS_INFO_STREAM("pose_relative_id->time: " << stamp->first);
//     ROS_INFO_STREAM("pose_relative_id->id: " << stamp->second);
//     ROS_INFO_STREAM("timestamp > pose_relative_id->time: " << left_a);
//   }
  
//   // Erase earlier timestampes
//   ROS_INFO_STREAM("timestamped_ids.size(): " << timestamped_ids.size());
//   timestamped_ids.erase(timestamped_ids.begin(), stamp);    
//   ROS_INFO_STREAM("timestamped_ids.size(): " << timestamped_ids.size());

//   return stamp->second;
// }


// int StereoHandler::searchCorrespondence(ros::Time stamp, std::map<double, int>& timestamped_ids)
// {
//   // Create a map iterator and point to the end of map
//   std::map<ros::Time, int>::reverse_iterator stamp_it = timestamped_ids.rbegin();


//   // int remove_it = 0;
//   // for (stamp_it = timestamped_ids.rbegin(); stamp_it != timestamped_ids.rend(); stamp_it++) { 
//   //   if ((stamp->first.sec > stamp.sec) || ((stamp->first.sec == stamp.sec) && (stamp->first.nsec > stamp.nsec)))
//   //   {
//   //     break;
//   //   } 
//   // } 

//   // timestamped_ids.erase(timestamped_ids.begin(), stamp);    

//   int remove_it = 0;
//   while ((stamp->first.sec > stamp.sec) || ((stamp->first.sec == stamp.sec) && (stamp->first.nsec > stamp.nsec)))
//   {
//     stamp++;
//     remove_it++;
//   }
  
//   timestamped_ids.erase(timestamped_ids.begin(), timestamped_ids.begin()+remove_it);    


//   // Evaluate new measurement timestamp to find where previous measurement from other factors took place

//   ROS_INFO_STREAM("timestamp: " << timestamp);
//   ROS_INFO_STREAM("pose_relative_id->time: " << stamp->first);
//   ROS_INFO_STREAM("pose_relative_id->id: " << stamp->second);
//   bool left_a = timestamp > stamp->first;
//   ROS_INFO_STREAM("timestamp > pose_relative_id->time: " << left_a);
  
//   while (stamp->first > timestamp)
//   {
//     stamp--;
//     left_a = timestamp > stamp->first;
//     ROS_INFO_STREAM("pose_relative_id->time: " << stamp->first);
//     ROS_INFO_STREAM("pose_relative_id->id: " << stamp->second);
//     ROS_INFO_STREAM("timestamp > pose_relative_id->time: " << left_a);
//   }
  
//   // Erase earlier timestampes
//   ROS_INFO_STREAM("timestamped_ids.size(): " << timestamped_ids.size());
//   timestamped_ids.erase(timestamped_ids.begin(), stamp);    
//   ROS_INFO_STREAM("timestamped_ids.size(): " << timestamped_ids.size());

//   return stamp->second;
// }



void StereoHandler::addPose2Graph(int& pose_id, std::map<double, int>& timestamped_ids, 
                                  const geometry_msgs::PoseStampedConstPtr &msg, gtsam::NonlinearFactorGraph& graph)
{
  double timestamp = msg->header.stamp.sec + (msg->header.stamp.nsec / 1e9);
  // int pose_relative_id = searchCorrespondence(timestamp, timestamped_ids);
  // int pose_relative_id = searchCorrespondence(msg->header.stamp.sec, msg->header.stamp.nsec, timestamped_ids);
  int pose_relative_id = pose_id;
  if (pose_id == pose_relative_id)
    timestamped_ids.insert(timestamped_ids.end(), std::pair<double,int>(timestamp, ++pose_id));

  _rotation = gtsam::Rot3::Quaternion(msg->pose.orientation.x,
                                      msg->pose.orientation.y,
                                      msg->pose.orientation.z,
                                      msg->pose.orientation.w);

  _translation = gtsam::Point3(msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z);

  graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id-1), 
                                                     gtsam::symbol_shorthand::X(pose_id), 
                                                     gtsam::Pose3(_rotation, _translation), 
                                                     _NOISE)); 
}





void StereoHandler::addCloud2Graph(int pose_id, const sensor_msgs::PointCloud2ConstPtr &cloud_msg, gtsam::NonlinearFactorGraph& graph)
{

}


