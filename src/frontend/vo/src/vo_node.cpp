/*** ROS packages ***/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

/*** OpenCV packages ***/
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

/*** C++ packages ***/
#include <string>

/*** Classes ***/
#include "vo/pinhole_model.h"
#include "support.h"
#include "vo/pose_prediction/pose_predictor.h"
#include "vo/feature_management.h"

class VO
{
  private:
    // Typedef
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    
    // Nodes
    ros::NodeHandle nh_; // TODO: Trenger man flere nodehandlers
    message_filters::Subscriber<sensor_msgs::Image> sub_cam_left_;
    message_filters::Subscriber<sensor_msgs::Image> sub_cam_right_;
    boost::shared_ptr<Sync> sync_;

    // Classes
    StereoCameras stereo_;
    Detector      detector_;
    PosePredictor pose_predictor_;

    // Parameters
    int tic_, toc_;

    cv::Mat prev_img_;
    std::vector<cv::KeyPoint> features_;
    std::vector<cv::KeyPoint> tracked_features_;

  public:
    VO() 
    : stereo_(nh_, 10),
      detector_(nh_),
      pose_predictor_(nh_, "imu_topic", 1000)
    {
      // Synchronization example: https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336
      sub_cam_left_.subscribe(nh_, "cam_left", 1);
      sub_cam_right_.subscribe(nh_, "cam_right", 1);
      sync_.reset(new Sync(MySyncPolicy(10), sub_cam_left_, sub_cam_right_));
      sync_->registerCallback(boost::bind(&VO::callback, this, _1, _2));
    }

    ~VO() {}


    void callback(const sensor_msgs::ImageConstPtr &cam_left, const sensor_msgs::ImageConstPtr &cam_right)
    {
      tic_ = cv::getTickCount();
      
      /***** Initialize *****/     
    
      //Frames
      cv_bridge::CvImagePtr cv_ptr_left;
      cv_bridge::CvImagePtr cv_ptr_right;
      try
      {
        cv_ptr_left  = cv_bridge::toCvCopy(cam_left, sensor_msgs::image_encodings::MONO8);
        cv_ptr_right = cv_bridge::toCvCopy(cam_right, sensor_msgs::image_encodings::MONO8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      stereo_.prepareImages(cv_ptr_left->image, cv_ptr_right->image);
      

      detector_.track(prev_img_, cv_ptr_left->image, features_);
      tracked_features_ = features_;
      detector_.bucketedFeatureDetection(cv_ptr_left->image, features_);
      
      displayWindowFeatures(cv_ptr_left->image, features_, cv_ptr_left->image, tracked_features_); 


      prev_img_ = cv_ptr_left->image;
      
      toc_ = cv::getTickCount();
      ROS_INFO_STREAM("Time per iteration: " <<  (toc_ - tic_)/ cv::getTickFrequency() << "\n");
    }

};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_frontend");
	ROS_INFO("\n\n\n ----- Starting VO frontend ----- \n");

	VO vo;

  ros::spin();
}