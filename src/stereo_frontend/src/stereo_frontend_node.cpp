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
// #include "opencv2/features2d.hpp"
// #include <opencv2/calib3d.hpp>

/*** C++ packages ***/
#include <string>

/*** Classes ***/
#include "stereo_frontend/feature_management.h"
#include "stereo_frontend/pinhole_model.h"
// #include "stereo_frontend/pose_estimation_eigen.h"
#include "stereo_frontend/pose_estimation_cv.h"
#include "stereo_frontend/support.h"
#include "stereo_frontend/point_cloud_management.h"
#include "motionBA/motionBA.h"


class StereoFrontend
{
  private:
    // Typedef
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    
    // Nodes
    ros::NodeHandle _nh; // TODO: Trenger man flere nodehandlers
    message_filters::Subscriber<sensor_msgs::Image> _sub_cam_left;
    message_filters::Subscriber<sensor_msgs::Image> _sub_cam_right;
    boost::shared_ptr<Sync> _sync;
    
  	ros::Subscriber _gnss_sub;
	  ros::Publisher  _cloud_pub; 
    ros::Publisher  _pose_relative_pub;
    ros::Publisher  _pose_world_pub;

    // Classes
    StereoCameras        _stereo;
    FeatureManager       _detector;
    PointCloudManager    _pcm;
    Pose                 _pose;
    Pose                 _ground_truth;
    Pose                 _ground_truth_kf;
    MotionBundleAdjuster _motionBA;

    // Parameters
    int _tic, _toc;
    bool _processed_first_frame, _initialized;

  public:
    StereoFrontend() 
    : _stereo(_nh, 10),
      _pose(_nh, "stamped_traj_estimate.txt"), _ground_truth(_nh, "stamped_groundtruth.txt"), _ground_truth_kf(_nh, "stamped_groundtruth.txt"),
      _detector(_nh, "camera_left", 10, 10, 25, 10), 
      _motionBA(_stereo.left().K_eig()),
      _processed_first_frame(false), _initialized(false)
    {
      // Synchronization example: https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336
      _sub_cam_left.subscribe(_nh, "cam_left", 1);
      _sub_cam_right.subscribe(_nh, "cam_right", 1);
      _sync.reset(new Sync(MySyncPolicy(10), _sub_cam_left, _sub_cam_right));
      _sync->registerCallback(boost::bind(&StereoFrontend::callback, this, _1, _2));

 			_gnss_sub  = _nh.subscribe("/tf", 1, &StereoFrontend::readTf, this);
      _cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("/frontend/point_cloud", 1000);
		  _pose_relative_pub  = _nh.advertise<geometry_msgs::PoseStamped>("/frontend/pose_relative", 1000);
		  _pose_world_pub     = _nh.advertise<geometry_msgs::PoseStamped>("/frontend/pose_world", 1000);
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
      
      // if (!_pose.isSet())
      //   _pose.setWorldCenter(_ground_truth.getWorldRotation(), _ground_truth.getWorldTranslation());

      _ground_truth.toFile();
    }



    void callback(const sensor_msgs::ImageConstPtr &cam_left, const sensor_msgs::ImageConstPtr &cam_right)
    {
      /***** Initialize *****/     
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

      _stereo.prepareImages(cv_ptr_left->image, cv_ptr_right->image);
      _detector.initiateFrames(cv_ptr_left->image, cv_ptr_right->image);


      if (_processed_first_frame) 
      {
        /***** Pose estimation *****/
        // Track features for initial pose estimate
        std::vector<cv::KeyPoint> tracked_prev = _detector.getPrevFeaturesLeft();
        std::vector<cv::KeyPoint> tracked_cur;
        if (! tracked_prev.empty())
        {
          _detector.track(_detector.getPrevImageLeft(), _detector.getCurImageLeft(), tracked_prev, tracked_cur);
        }

        std::vector<cv::Point2f> points_prev, points_cur;
        cv::KeyPoint::convert(tracked_prev, points_prev);
        cv::KeyPoint::convert(tracked_cur, points_cur);
        
        _pose.initialPoseEstimate(points_prev, points_cur, _stereo.left().K_cv());
        _pose.updatePose();


        // Refine pose estimate
        if (_initialized)
        {
          PoseEstimate init_pose_estimate(points_cur, _pcm.getWorldPoints(), _pose.getRelativeTransformation());

          ROS_INFO("-------------------------------------------------");
          ROS_INFO_STREAM("init_pose_estimate.linear(): \n" << init_pose_estimate.T_wb.linear());
          ROS_INFO_STREAM("init_pose_estimate.translation(): \n" << init_pose_estimate.T_wb.translation());

          init_pose_estimate = _motionBA.estimate(init_pose_estimate);
        
          ROS_INFO_STREAM("refined.linear(): \n" << init_pose_estimate.T_wb.linear());
          ROS_INFO_STREAM("refined.translation(): \n" << init_pose_estimate.T_wb.translation());
          ROS_INFO("-------------------------------------------------");
        }
        
      }


      /***** Feature management *****/
      // Manage features for stereo analysis
      _detector.trackBuckets();
      _detector.bucketedFeatureDetection(true);    

      ROS_INFO_STREAM("Detect - number of features: " << _detector.getNumFeaturesLeftCur());
      displayWindowFeatures(_detector.getCurImageLeft(), _detector.getCurFeaturesLeft());


      /***** Point management *****/
      if (_processed_first_frame) 
      {
        // Match features for triangulation
        std::vector<cv::KeyPoint> match_left, match_right;
        _detector.circularMatching(match_left, match_right);

        _pcm.triangulate(match_left, match_right, _stereo.leftProjMat(), _stereo.rightProjMat(), _pose.getWorldRotation(), _pose.getWorldTranslation());


        /***** Publish *****/
        _pcm.setPointCloudHeader(cam_left->header);
        _cloud_pub.publish(_pcm.toPointCloud2Msg(cam_left->header));
        _pose_relative_pub.publish(_pose.toPoseStamped(cam_left->header, _pose.getRelativeTransformation()));  
        _pose_world_pub.publish(_pose.toPoseStamped(cam_left->header, _pose.getWorldTransformation()));    

        /***** End-of-iteration updates *****/
        _pose.toFile();

        _initialized = true;
      }

      _detector.updatePrevFrames(); 
      _processed_first_frame = true;

      _toc = cv::getTickCount();
      ROS_INFO_STREAM("Time per iteration: " <<  (_toc - _tic)/ cv::getTickFrequency() << "\n");
    }

};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_frontend");
	ROS_INFO("\n\n\n ----- Starting VO frontend ----- \n");

  // Node synchronizer;
	StereoFrontend callback;

  ros::spin();
}