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


/*** Typedef ***/
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;



class StereoFrontend
{
  private:
    // Nodes
    ros::NodeHandle _nh;
    message_filters::Subscriber<sensor_msgs::Image> _sub_cam_left;
    message_filters::Subscriber<sensor_msgs::Image> _sub_cam_right;
    boost::shared_ptr<Sync> _sync;
    
    // Classes
	  FeatureManager _detector;
    PinholeModel   _camera_left;
    PinholeModel   _camera_right;
    Pose           _pose;
    Pose           _ground_truth;

    // Parameters
    int tic, toc;

  public:
    StereoFrontend() 
    : _camera_left(_nh, "camera_left"), _camera_right(_nh, "camera_right"),
      _pose("stamped_traj_estimate.txt"), _ground_truth("stamped_groundtruth.txt"),
      _detector(_nh, "camera_left", 10, 10, 25, 10)
    {
      // Synchronization example: https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336
      _sub_cam_left.subscribe(_nh, "cam_left", 1);
      _sub_cam_right.subscribe(_nh, "cam_right", 1);
      _sync.reset(new Sync(MySyncPolicy(10), _sub_cam_left, _sub_cam_right));
      _sync->registerCallback(boost::bind(&StereoFrontend::callback, this, _1, _2));

    }

    ~StereoFrontend()
    {
      // cv::destroyWindow(OPENCV_WINDOW);
    }


    void callback(const sensor_msgs::ImageConstPtr &cam_left, const sensor_msgs::ImageConstPtr &cam_right)
    {
      int tic = cv::getTickCount();

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
      /***** Initiate *****/
      _camera_left.undistort(cv_ptr_left->image);
      _camera_right.undistort(cv_ptr_right->image);

      int patch_width = _camera_left.getWidth() - (_camera_left.getWidth() % _detector.getGridSize());
      int patch_height = _camera_left.getHeight() - (_camera_left.getHeight() % _detector.getGridSize());
      _camera_left.crop(cv_ptr_left->image, 0, 0, patch_width, patch_height);
      _camera_right.crop(cv_ptr_right->image, 0, 0, patch_width, patch_height);

    	_detector.initiateFrames(cv_ptr_left->image, cv_ptr_right->image);

      /***** Feature management *****/
      _detector.trackBuckets();
      _detector.bucketedFeatureDetection(true);    
      ROS_INFO_STREAM("Detect - number of features: " << _detector.getNumFeaturesLeftCur());

      displayWindowFeatures(_detector.getCurImageLeft(), _detector.getCurFeaturesLeft());


      std::vector<cv::KeyPoint> features_left, features_right;
      _detector.circularMatching(features_left, features_right);

      ROS_INFO_STREAM("Match - number of features: " << features_left.size());
      ROS_INFO_STREAM("Match - number of features: " << features_right.size());

      displayWindowFeatures(_detector.getCurImageLeft(), features_left, _detector.getCurImageRight(), features_right, "Matches");

      /***** Point management *****/


      /***** End-of-iteration updates *****/
      _detector.updatePrevFrames();

      toc = cv::getTickCount();
      ROS_INFO_STREAM("Time per iteration: " <<  (toc - tic)/ cv::getTickFrequency() << "\n");
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




