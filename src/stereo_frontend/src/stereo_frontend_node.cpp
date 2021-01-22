/*** ROS packages ***/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>

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
#include "stereo_frontend/pose_estimation.h"
#include "stereo_frontend/support.h"
#include "stereo_frontend/point_cloud_management.h"

/*** Typedef ***/
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;



class StereoFrontend
{
  private:
    // Nodes
    ros::NodeHandle _nh; // TODO: Trenger man flere nodehandlers
    message_filters::Subscriber<sensor_msgs::Image> _sub_cam_left;
    message_filters::Subscriber<sensor_msgs::Image> _sub_cam_right;
    boost::shared_ptr<Sync> _sync;
    
  	ros::Subscriber _gnss_sub;

    // Classes
    // StereoCameras     _stereo_cameras;
    PinholeModel      _camera_left;
    PinholeModel      _camera_right;
    FeatureManager    _detector;
    PointCloudManager _pcm;
    Pose              _pose;
    Pose              _ground_truth;
    Pose              _ground_truth_kf;

    // Parameters
    int _tic, _toc;
    bool _initialized;

  public:
    StereoFrontend() 
    : _camera_left(_nh, "camera_left"), _camera_right(_nh, "camera_right"),
      _pose(_nh, "stamped_traj_estimate.txt"), _ground_truth(_nh, "stamped_groundtruth.txt"), _ground_truth_kf(_nh, "stamped_groundtruth.txt"),
      _detector(_nh, "camera_left", 10, 10, 25, 10), 
      _initialized(false)
    {
      // Synchronization example: https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336
      _sub_cam_left.subscribe(_nh, "cam_left", 1);
      _sub_cam_right.subscribe(_nh, "cam_right", 1);
      _sync.reset(new Sync(MySyncPolicy(10), _sub_cam_left, _sub_cam_right));
      _sync->registerCallback(boost::bind(&StereoFrontend::callback, this, _1, _2));

 			_gnss_sub = _nh.subscribe("/tf", 1, &StereoFrontend::readTf, this);

    }

    ~StereoFrontend()
    {
      // cv::destroyWindow(OPENCV_WINDOW);
    }


    void readTf(const tf2_msgs::TFMessage& msg)
    {
      double timestamp = msg.transforms[0].header.stamp.sec + (msg.transforms[0].header.stamp.nsec / 1e9);
      _ground_truth.readTimestamp(timestamp);

      _ground_truth.readTranslation(msg.transforms[0].transform.translation.x,
                                    msg.transforms[0].transform.translation.y,
                                    msg.transforms[0].transform.translation.z);

      _ground_truth.readRotation(msg.transforms[0].transform.rotation.x,
                                 msg.transforms[0].transform.rotation.y,
                                 msg.transforms[0].transform.rotation.z,
                                 msg.transforms[0].transform.rotation.w);

      _ground_truth.correctGNSSFrame();
      
      _ground_truth.toFile();
    }


    void callback(const sensor_msgs::ImageConstPtr &cam_left, const sensor_msgs::ImageConstPtr &cam_right)
    {
      /***** Initialize *****/
      if (!_initialized)
      {
        init_cb(cam_left, cam_right);
        _initialized = true;
      }
      else
      {         
        // Ground truth      
        _tic = cv::getTickCount();

        _pose.readTimestamp(ros::Time::now().toSec()); 
        _pose.estimateScaleFromGNSS(_ground_truth.getWorldTranslation(), _ground_truth_kf.getWorldTranslation());

        _ground_truth_kf = _ground_truth;

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

        _camera_left.rectify(cv_ptr_left->image);
        _camera_right.rectify(cv_ptr_right->image);

        int patch_width = _camera_left.getWidth() - (_camera_left.getWidth() % _detector.getGridSize());
        int patch_height = _camera_left.getHeight() - (_camera_left.getHeight() % _detector.getGridSize());
        _camera_left.crop(cv_ptr_left->image, 0, 0, patch_width, patch_height);
        _camera_right.crop(cv_ptr_right->image, 0, 0, patch_width, patch_height);

        _detector.initiateFrames(cv_ptr_left->image, cv_ptr_right->image);


        /***** Feature management *****/
        // Track features for initial pose estimate
        std::vector<cv::KeyPoint> tracked_prev = _detector.getPrevFeaturesLeft();
        std::vector<cv::KeyPoint> tracked_cur;
        if (! tracked_prev.empty())
          _detector.track(_detector.getPrevImageLeft(), _detector.getCurImageLeft(), tracked_prev, tracked_cur);
        
        // Manage features for stereo analysis
        _detector.trackBuckets();
        _detector.bucketedFeatureDetection(true);    

        ROS_INFO_STREAM("Detect - number of features: " << _detector.getNumFeaturesLeftCur());
        displayWindowFeatures(_detector.getCurImageLeft(), _detector.getCurFeaturesLeft());

        // Match features
        std::vector<cv::KeyPoint> match_left, match_right;
        _detector.circularMatching(match_left, match_right);

        ROS_INFO_STREAM("Match - number of features: " << match_left.size());
        displayWindowFeatures(_detector.getCurImageLeft(), match_left, _detector.getCurImageRight(), match_right, "Matches");


        /***** Pose estimation *****/
        std::vector<cv::Point2f> points_prev, points_cur;
        cv::KeyPoint::convert(tracked_prev, points_prev);
        cv::KeyPoint::convert(tracked_cur, points_cur);
        
        bool valid_pose = _pose.initialPoseEstimate(points_prev, points_cur, _camera_left.K_cv());
        _pose.updatePose();

        // ROS_INFO_STREAM("World rotation: " << _pose.getWorldRotation() );
        // ROS_INFO_STREAM("World translation: " << _pose.getWorldTranslation() );
    

        /***** Point management *****/
        _pcm.triangulate(match_left,
                         match_right,
                         _camera_left.K_cv(),
                         _camera_right.K_cv(),
                         _camera_right.getStereoRotation(),
                         _camera_right.getStereoTranslation());

        // TODO: MLPnP
        // _pcm.setExtrinsics(_pose.getWorldRotation(), _pose.getWorldTranslation());



        /***** End-of-iteration updates *****/
        _pose.toFile();
        // _ground_truth_kf.toFile();

        _detector.updatePrevFrames();

        _toc = cv::getTickCount();
        ROS_INFO_STREAM("Time per iteration: " <<  (_toc - _tic)/ cv::getTickFrequency() << "\n");
      }
    }


    // To initialize: basicly same as main loop
    void init_cb(const sensor_msgs::ImageConstPtr &cam_left, const sensor_msgs::ImageConstPtr &cam_right)
    {
      _ground_truth_kf = _ground_truth;

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

      _camera_left.rectify(cv_ptr_left->image);
      _camera_right.rectify(cv_ptr_right->image);

      int patch_width = _camera_left.getWidth() - (_camera_left.getWidth() % _detector.getGridSize());
      int patch_height = _camera_left.getHeight() - (_camera_left.getHeight() % _detector.getGridSize());
      _camera_left.crop(cv_ptr_left->image, 0, 0, patch_width, patch_height);
      _camera_right.crop(cv_ptr_right->image, 0, 0, patch_width, patch_height);

    	_detector.initiateFrames(cv_ptr_left->image, cv_ptr_right->image);


      /***** Feature management *****/
      // Manage features for stereo analysis
      _detector.trackBuckets();
      _detector.bucketedFeatureDetection(true);    

      ROS_INFO_STREAM("Detect - number of features: " << _detector.getNumFeaturesLeftCur());
      displayWindowFeatures(_detector.getCurImageLeft(), _detector.getCurFeaturesLeft());

      // Match features
      // std::vector<cv::KeyPoint> match_left, match_right;
      // _detector.circularMatching(match_left, match_right);

      // ROS_INFO_STREAM("Match - number of features: " << match_left.size());
      // displayWindowFeatures(_detector.getCurImageLeft(), match_left, _detector.getCurImageRight(), match_right, "Matches");


      /***** End-of-iteration updates *****/
      _detector.updatePrevFrames();    
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




