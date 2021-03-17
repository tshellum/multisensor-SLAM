/*** ROS packages ***/
#include <ros/ros.h>
#include <ros/package.h>
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
#include "support.h"
#include "vo/pinhole_model.h"
#include "vo/pose_prediction/pose_predictor.h"
#include "vo/feature_management.h"
#include "PYR/PYR.h"
// #include "JET/jet.h"

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
    // PosePredictor pose_predictor_;
    PYR           pyr_;
    // JET jet;

    // Parameters
    bool initialized_;
    ros::Time stamp_img_k_;
    int tic_, toc_;

    cv::Mat prev_img;
    std::vector<cv::KeyPoint> features_;
    std::vector<cv::KeyPoint> tracked_features_;

    std::string config_path_;

  public:
    VO() 
    : config_path_(ros::package::getPath("vo") + "/../../../config/kitti/"),
      initialized_(false),
      stereo_(nh_, 10),
      detector_(nh_)
      // pose_predictor_(nh_, "imu_topic", 1000)
      // pyr_(readConfigFromJsonFile( config_path_ + "PYR.json" ), stereo_.left().K_cv() )
      // jet(config_path_)
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
      
      if (initialized_)
      {
        double dt = ( cam_left->header.stamp - stamp_img_k_ ).toSec();
        // pose_predictor_.predict(dt);
      }
      std::pair<cv::Mat, cv::Mat> images = stereo_.storeImagePairGray(cam_left, cam_right);
      cv::Mat img_left  = images.first;
      cv::Mat img_right = images.second;
      stereo_.prepareImages(img_left, img_right);
      

      detector_.track(prev_img, img_left, features_);
      // detector_.track(stereo_.left().getPreviousImage(), img_left, features_);
      tracked_features_ = features_;
      detector_.bucketedFeatureDetection(img_left, features_);
      
      // pyr_.Estimate(prev_img, img_left);
      // cv::Mat img_current_disp2 = pyr_.Draw(img_left);		
      // imshow("results", img_current_disp2);

      // displayWindowFeatures(img_left, features_, img_left, tracked_features_); 


      initialized_ = true;
      cv::Mat prev_img = img_left;
      stamp_img_k_ = cam_left->header.stamp;

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