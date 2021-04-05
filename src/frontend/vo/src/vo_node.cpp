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
#include "vo/VO_msg.h"


/*** OpenCV packages ***/
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

/*** C++ packages ***/
#include <string>

/*** Classes ***/
#include "support.h"
#include "inputOutput.h"
#include "vo/pinhole_model.h"
#include "vo/pose_prediction/pose_predictor.h"
#include "vo/feature_management/detector.h"
#include "vo/feature_management/matcher.h"
#include "vo/sequencer.h"
#include "BA/gtsam/structure-only_BA.h"
#include "BA/gtsam/motion-only_BA.h"
// #include "PYR/PYR.h"
// #include "JET/jet.h"

class VO
{
  private:
    // Typedef
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    
    // Subscriber
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> sub_cam_left_;
    message_filters::Subscriber<sensor_msgs::Image> sub_cam_right_;
    boost::shared_ptr<Sync> sync_;

    // Publisher
    ros::Publisher vo_pub_;

    // Classes
    Sequencer               sequencer_;
    StereoCameras           stereo_;
    Detector                detector_;
    Matcher                 matcher_;
    PosePredictor           pose_predictor_;
    BA::StructureEstimator  structure_BA_;
    BA::MotionEstimator     motion_BA_;
    // PYR           pyr_;
    // JET jet;

    // Parameters
    bool initialized_;
    ros::Time stamp_img_k_;
    int tic_, toc_;

    std::vector<cv::KeyPoint> features_;
    std::vector<cv::KeyPoint> tracked_features_;

    const std::string config_path_;
    const boost::property_tree::ptree camera_config;
    const boost::property_tree::ptree detector_config;

    std::string frame_;
  public:
    VO() 
    : initialized_(false)
    , stereo_(nh_, 10)
    , pose_predictor_(nh_, "imu_topic", 1000)
    , structure_BA_(stereo_.left().K_eig(), stereo_.right().K_eig(), stereo_.getInverseStereoTransformation(), 0.5)
    , motion_BA_(stereo_.left().K_eig(), stereo_.getStereoTransformation(), 0.1, M_PI/9, 1)
    {
      nh_.getParam("/frame", frame_);

      // Synchronization example: https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336
      sub_cam_left_.subscribe(nh_, "cam_left", 1);
      sub_cam_right_.subscribe(nh_, "cam_right", 1);
      sync_.reset(new Sync(MySyncPolicy(10), sub_cam_left_, sub_cam_right_));
      sync_->registerCallback(boost::bind(&VO::callback, this, _1, _2));

      // Publish
		  vo_pub_  = nh_.advertise<vo::VO_msg>("/frontend/vo", 1000);

      // Construct classes
      const std::string config_path_ = ros::package::getPath("vo") + "/../../../config/kitti/";

      detector_ = Detector( readConfigFromJsonFile( config_path_ + "feature_management.json" ),
                            readConfigFromJsonFile( config_path_ + "camera.json" ) );
      matcher_  = Matcher( readConfigFromJsonFile( config_path_ + "feature_management.json" ) );
      // pyr_ = PYR( readConfigFromJsonFile( config_path_ + "PYR.json" ), stereo_.left().K_cv() );
      // jet_ = JET( readConfigFromJsonFile( config_path_ + "JET.json" ) );

    }

    ~VO() {}


    void callback(const sensor_msgs::ImageConstPtr &cam_left, const sensor_msgs::ImageConstPtr &cam_right)
    {
      ROS_INFO("-------------------------------------------------------");

      tic_ = cv::getTickCount();
      
      if (initialized_)
      {
        double dt = ( cam_left->header.stamp - stamp_img_k_ ).toSec();
        // pose_predictor_.predict(dt);
      }



      /***** Preprocess and store image *****/
      std::pair<cv::Mat, cv::Mat> images = stereo_.preprocessImages(readGray(cam_left), 
                                                                    readGray(cam_right));

      sequencer_.storeImagePair(images.first, 
                                images.second);
      
      

      /***** Track features and estimate relative pose *****/
      sequencer_.current.kpts_l = matcher_.projectLandmarks(sequencer_.previous.world_points,
                                                            pose_predictor_.getPoseRelative(),
                                                            stereo_.left().K_eig(),
                                                            sequencer_.previous.kpts_l);

      ROS_INFO_STREAM("projected - previous.kpts_l: " << sequencer_.previous.kpts_l.size() << " - current.kpts_l: " << sequencer_.current.kpts_l.size());


      matcher_.track(sequencer_.previous.img_l, 
                     sequencer_.current.img_l, 
                     sequencer_.previous.kpts_l,
                     sequencer_.current.kpts_l);

      ROS_INFO_STREAM("tracked - previous.kpts_l: " << sequencer_.previous.kpts_l.size() << " - current.kpts_l: " << sequencer_.current.kpts_l.size());


      sequencer_.current.T_r = pose_predictor_.estimatePoseFromFeatures(sequencer_.previous.kpts_l, 
                                                                        sequencer_.current.kpts_l, 
                                                                        stereo_.left().K_cv());

      if ( ! pose_predictor_.evaluateValidity(sequencer_.current.T_r, sequencer_.previous.T_r) )
        sequencer_.current.T_r = sequencer_.previous.T_r;

      sequencer_.current.T_r.translation() *= sequencer_.previous.scale;



      /***** Manage new features *****/
      detector_.bucketedFeatureDetection(sequencer_.current.img_l, 
                                         sequencer_.current.kpts_l);
      
      ROS_INFO_STREAM("detected - kpts_l.size(): " << sequencer_.current.kpts_l.size());


      std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> stereo_features = matcher_.circularMatching(sequencer_.current.img_l,
                                                                                                                  sequencer_.current.img_r,
                                                                                                                  sequencer_.previous.img_l,
                                                                                                                  sequencer_.previous.img_r, 
                                                                                                                  sequencer_.current.kpts_l, 
                                                                                                                  sequencer_.current.kpts_r);

      sequencer_.storeFeatures(stereo_features.first, stereo_features.second);

      ROS_INFO_STREAM("matched - match_left.size(): " << stereo_features.first.size() << ", match_right.size(): " << stereo_features.second.size());


      std::pair<std::vector<cv::Point3f>, std::vector<int>> wrld_pts = matcher_.triangulate(sequencer_.current.kpts_l, 
                                                                                            sequencer_.current.kpts_r, 
                                                                                            stereo_.leftProjMat(), 
                                                                                            stereo_.rightProjMat());
      sequencer_.storeCloud(wrld_pts.first,
                            wrld_pts.second);



      /***** Refine results using BA *****/
      std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> img_pts = convert(sequencer_.current.kpts_l, 
                                                                                      sequencer_.current.kpts_r);

      sequencer_.current.world_points = structure_BA_.estimate(sequencer_.current.world_points,
                                                               img_pts.first,
                                                               img_pts.second);

      std::vector<cv::Point3f> landmarks_prev;
      std::vector<cv::Point2f> features_cur_l, features_cur_r;
      find3D2DCorrespondences(sequencer_.previous.world_points,
                              sequencer_.previous.indices,
                              sequencer_.current.kpts_l,
                              sequencer_.current.kpts_r,
                              landmarks_prev,
                              features_cur_l,
                              features_cur_r);
      
      Eigen::Affine3d T_r_opt = motion_BA_.estimate(sequencer_.current.T_r,
                                                    landmarks_prev,
                                                    features_cur_l,
                                                    features_cur_r);

      // ROS_INFO_STREAM("Motion-BA using " << landmarks_prev.size() << " points - Optimized pose: \n" << T_r_opt.matrix());

      double scale_cur = pose_predictor_.calculateScale(T_r_opt.translation(), 
                                                        sequencer_.previous.scale);


      /***** End of iteration processes *****/
      displayWindowFeatures(sequencer_.current.img_l, 
                            stereo_features.first,
                            sequencer_.current.img_r,
                            stereo_features.second,
                            "Feature matches",
                            1920,
                            1080); 
      
      // Rejecting bad pose optimization --> Setting equal to previous
      if ( (T_r_opt.matrix() == Eigen::Matrix4d::Identity()) // motion-BA actually produces an estimate
        || (scale_cur > 10)                                  // No large motion
        || (scale_cur < 0.1)                                 // Standing still - this is typically where thing goes wrong
        || (T_r_opt(2,3) < -0.1)                             // No large backward motion
      )                                
      {
        sequencer_.current.scale = sequencer_.previous.scale;
      }
      else
      {
        sequencer_.current.T_r = T_r_opt;
        sequencer_.current.scale = scale_cur;
      }

      ROS_INFO_STREAM("VO - calculated pose: \n" << sequencer_.current.T_r.matrix());

      vo_pub_.publish( generateMsgInBody(cam_left->header.stamp, 
                                         sequencer_.current.T_r,
                                         sequencer_.current.world_points,
                                         sequencer_.current.indices,
                                         frame_) );


      sequencer_.updatePreviousFrame();
      initialized_ = true;
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