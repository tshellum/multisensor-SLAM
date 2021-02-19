/*** ROS packages ***/
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

/*** GTSAM packages ***/
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/inference/Symbol.h> 


class StereoHandler
{
private:
  // double _timestamp;
  // std::vector<double> _timestamps;
  int _prev_odometry_id;
  int _cur_odometry_id;
  
  gtsam::Rot3   _rotation;
  gtsam::Point3 _translation;
  gtsam::noiseModel::Diagonal::shared_ptr _NOISE;
public:
  StereoHandler() : _prev_odometry_id(0), _cur_odometry_id(0)
  {
    gtsam::Vector6 sigmas;
    sigmas << gtsam::Vector3(1.0, 1.0, 1.0), gtsam::Vector3(1.0, 1.0, 1.0);
    _NOISE = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  };
  ~StereoHandler() {};


  void setCurrentPoseID(int id) {_cur_odometry_id = id;};

  void addPose2Graph(const geometry_msgs::PoseStampedConstPtr &pose_msg, gtsam::NonlinearFactorGraph& graph);

  void addCloud2Graph(int pose_id,  
                      const sensor_msgs::PointCloud2ConstPtr &cloud_msg, gtsam::NonlinearFactorGraph& graph);
};



void StereoHandler::addPose2Graph(const geometry_msgs::PoseStampedConstPtr &msg, 
                                  gtsam::NonlinearFactorGraph& graph)
{
  if (_cur_odometry_id == _prev_odometry_id)
    _cur_odometry_id++;

  _rotation = gtsam::Rot3::Quaternion(msg->pose.orientation.x,
                                      msg->pose.orientation.y,
                                      msg->pose.orientation.z,
                                      msg->pose.orientation.w);

  _translation = gtsam::Point3(msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z);

  graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(_prev_odometry_id), 
                                                     gtsam::symbol_shorthand::X(_cur_odometry_id), 
                                                     gtsam::Pose3(_rotation, _translation), 
                                                     _NOISE)); 

  _prev_odometry_id = _cur_odometry_id;
}


void StereoHandler::addCloud2Graph(int pose_id, const sensor_msgs::PointCloud2ConstPtr &cloud_msg, gtsam::NonlinearFactorGraph& graph)
{

}

