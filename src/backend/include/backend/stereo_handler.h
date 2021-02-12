/*** ROS packages ***/
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

/*** GTSAM packages ***/
#include <gtsam/slam/BetweenFactor.h>



class StereoHandler
{
private:
  double _timestamp;

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

  void addPose2Graph(int pose_id, const geometry_msgs::PoseStampedConstPtr &pose_msg, gtsam::NonlinearFactorGraph& graph);
  void addCloud2Graph(int pose_id, const sensor_msgs::PointCloud2ConstPtr &cloud_msg, gtsam::NonlinearFactorGraph& graph);

};



void StereoHandler::addPose2Graph(int pose_id, const geometry_msgs::PoseStampedConstPtr &msg, gtsam::NonlinearFactorGraph& graph)
{
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


