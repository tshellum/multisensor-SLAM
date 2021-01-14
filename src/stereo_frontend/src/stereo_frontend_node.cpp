/*** ROS libraries ***/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
// #include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

/*** OpenCV libraries ***/
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
// #include "opencv2/features2d.hpp"
// #include <opencv2/calib3d.hpp>

/*** C++ libraries ***/
#include <string>

/*** Classes ***/
#include "stereo_frontend/feature_management.h"
#include "stereo_frontend/pinhole_model.h"


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
    
    // TODO: Read camera name
    PinholeModel camera_left_;
    // PinholeModel camera_left_("camera_left");
    // PinholeModel   camera_right_("camera_right");

    // Parameters

  public:
    StereoFrontend()
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

    	Frame frame_left;
      frame_left.image = cv_ptr_left->image; 
      frame_left.descriptor = cv::Mat(); 
      frame_left.keypoints.clear(); 

      Frame frame_right;
      frame_right.image = cv_ptr_right->image; 
      frame_right.descriptor = cv::Mat(); 
      frame_right.keypoints.clear(); 

      displayWindow(frame_left.image, frame_right.image);

 			// detector_.detectAndCompute(frame_left);
 			// detector_.detectAndCompute(frame_right);

      // displayWindow(frame_left.image, frame_right.image, "Detections");

      ROS_INFO_STREAM("\nK: " << camera_left_.get_K() );
    }



    /*** Supporting functions ***/

    void displayWindow(cv::Mat image1, cv::Mat image2=cv::Mat(), std::string name="Stereo images", int resizeWidth=1000, int resizeHeight=500, int key=3)
    {
      if (image2.empty())
      {
        cv::namedWindow(name, cv::WINDOW_NORMAL);
        cv::imshow(name, image1);
        cv::resizeWindow(name, resizeWidth, resizeHeight);
        cv::waitKey(key);
      }
      else
      {
        cv::Mat splitimage;
        cv::hconcat(image1, image2, splitimage);
        
        cv::namedWindow(name, cv::WINDOW_NORMAL);
        cv::imshow(name, splitimage);
        cv::resizeWindow(name, resizeWidth, resizeHeight);
        cv::waitKey(key);
      }
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




