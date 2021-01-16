/*** ROS packages ***/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
// #include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

/*** OpenCV packages ***/
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
// #include "opencv2/features2d.hpp"
// #include <opencv2/calib3d.hpp>

/*** C++ packages ***/
#include <string>

/*** Classes ***/
#include "stereo_frontend/feature_management.h"
#include "stereo_frontend/pinhole_model.h"
#include "stereo_frontend/pose.h"
#include "stereo_frontend/support.h"


class StereoFrontend
{
  private:
    // Nodes
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> sub_cam_left_;
    message_filters::Subscriber<sensor_msgs::Image> sub_cam_right_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    
    // Classes
	  FeatureManager detector_;
    PinholeModel   camera_left_;
    PinholeModel   camera_right_;
    Pose           pose_;
    Pose           ground_truth_;

    // Parameters

  public:
    StereoFrontend() 
    : camera_left_("camera_left", nh_), camera_right_("camera_right", nh_),
      pose_("stamped_traj_estimate.txt"), ground_truth_("stamped_groundtruth.txt"),
      detector_(1000, 20, 25)
    {
      // Synchronization example: https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336
      sub_cam_left_.subscribe(nh_, "cam_left", 1);
      sub_cam_right_.subscribe(nh_, "cam_right", 1);
      sync_.reset(new Sync(MySyncPolicy(10), sub_cam_left_, sub_cam_right_));
      sync_->registerCallback(boost::bind(&StereoFrontend::callback, this, _1, _2));

    }

    ~StereoFrontend()
    {
      // cv::destroyWindow(OPENCV_WINDOW);
    }


    void callback(const sensor_msgs::ImageConstPtr &cam_left, const sensor_msgs::ImageConstPtr &cam_right)
    {
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

      // TODO: Formulate main system structure
      camera_left_.undistort(cv_ptr_left->image);
      camera_right_.undistort(cv_ptr_right->image);

      // camera_left_.crop(cv_ptr_left->image, 0, 0, cv_ptr_left->image.cols/2, cv_ptr_left->image.rows);
      // camera_right_.crop(cv_ptr_right->image, 0, 0, cv_ptr_right->image.cols/2, cv_ptr_right->image.rows);

    	detector_.initiate_frames(cv_ptr_left->image, cv_ptr_right->image);
 			detector_.detectAndCompute();

      displayWindowKeypoints(detector_.get_image_left(), detector_.get_image_right(), detector_.get_keypoints_left(), detector_.get_keypoints_right());
    

    
    
    }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_frontend");
	ROS_INFO("\n\n\n ----- Starting VO frontend ----- \n");

  // Node synchronizer;
	StereoFrontend callback;

  ros::spin();
  return 0;
}




