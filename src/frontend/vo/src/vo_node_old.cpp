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
#include "vo/loop_detector.h"
#include "vo/stereo_pinhole_model.h"


std::string getParam(ros::NodeHandle nh, std::string param)
{
  std::string out;
  nh.getParam(param, out);
  return out;
}



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

  // Parameters
  bool initialized_;
  int tic_, toc_;

  bool is_keyframe_;
  int sequence_id_;
  int keyframe_id_;

  const std::string base_path_, config_path_;
  const boost::property_tree::ptree camera_config;
  const boost::property_tree::ptree detector_config;

  std::string frame_;

  Eigen::Affine3d T_kf_;

  // Classes
  Sequencer               sequencer_;
  StereoCameras           stereo_;
  StereoPinholeModel      stereo_model_;
  Detector                detector_;
  Matcher                 matcher_;
  PosePredictor           pose_predictor_;
  BA::StructureEstimator  structure_BA_;
  BA::MotionEstimator     motion_BA_;
  // PYR           pyr_;
  // JET jet;
  LoopDetector            loop_detector_;



public:
  VO() 
  : initialized_(false)
  , is_keyframe_(false)
  , sequence_id_(0)
  , keyframe_id_(0)
  , T_kf_(Eigen::Affine3d::Identity())
  , base_path_( ros::package::getPath("vo") + "/../../../" )
  , config_path_( base_path_ + "config/" + getParam(nh_, "/dataset") + "/" )
  , camera_config( readConfigFromJsonFile( config_path_ + "camera.json" ) )
  , stereo_(nh_, 
            10)
  , stereo_model_( readConfigFromJsonFile( config_path_ + "camera.json" ),
                   readConfigFromJsonFile( config_path_ + "feature_management.json" ) )
  , pose_predictor_(nh_, 
                    "imu_topic", 
                    1000)
  , structure_BA_(stereo_.left().K_eig(), 
                  stereo_.right().K_eig(), 
                  stereo_.getInverseStereoTransformation(), 
                  0.5)
  , motion_BA_(stereo_.left().K_eig(), 
               stereo_.getStereoTransformation(), 
               0.1, 
               M_PI/9, 
               1)
  , loop_detector_(ros::package::getPath("vo") + "/../../../vocabulary/ORBvoc.bin",
                   stereo_.left().getWidth(),
                   stereo_.left().getHeight())
  {
    // Synchronization example: https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336
    sub_cam_left_.subscribe(nh_, "cam_left", 1);
    sub_cam_right_.subscribe(nh_, "cam_right", 1);
    sync_.reset(new Sync(MySyncPolicy(10), sub_cam_left_, sub_cam_right_));
    sync_->registerCallback(boost::bind(&VO::callback, this, _1, _2));

    // Publish
    vo_pub_  = nh_.advertise<vo::VO_loop_msg>("/frontend/vo", 1000);

    // Read base parameters
    nh_.getParam("/frame", frame_);


    // Construct classes
    std::string dataset;
    nh_.getParam("/dataset", dataset);
    const std::string base_path = ros::package::getPath("vo") + "/../../../";
    const std::string config_path = base_path + "config/" + dataset + "/";

    detector_ = Detector( readConfigFromJsonFile( config_path_ + "feature_management.json" ),
                          readConfigFromJsonFile( config_path_ + "camera.json" ) );
    matcher_  = Matcher( readConfigFromJsonFile( config_path_ + "feature_management.json" ),
                          detector_.getDefaultNorm() );

    
    #ifdef OPENCV_CUDA_ENABLED
      ROS_INFO("CUDA for OpenCV=ON");
    #else
      ROS_INFO("CUDA for OpenCV=OFF");
    #endif

    ROS_INFO("Visual odometry constructed...\n");
  }

  ~VO() {}


  void callback(const sensor_msgs::ImageConstPtr &cam_left, const sensor_msgs::ImageConstPtr &cam_right)
  {
    tic_ = cv::getTickCount();

    /***** Preprocess and store image *****/
    stereo_model_.preprocessImagePair(readGray(cam_left), 
                                      readGray(cam_right));

    std::pair<cv::Mat, cv::Mat> images = stereo_.preprocessImages(readGray(cam_left), 
                                                                  readGray(cam_right));

    sequencer_.storeImagePair(images.first, 
                              images.second);
    
    

    /***** Track features and estimate relative pose *****/
    sequencer_.current.kpts_l = matcher_.projectLandmarks(sequencer_.previous.world_points,
                                                          pose_predictor_.getPoseRelative(),
                                                          stereo_.left().K_eig(),
                                                          sequencer_.previous.kpts_l);

    // ROS_INFO_STREAM("projected - previous.kpts_l: " << sequencer_.previous.kpts_l.size() << " - current.kpts_l: " << sequencer_.current.kpts_l.size());


    matcher_.track(sequencer_.previous.img_l, 
                    sequencer_.current.img_l, 
                    sequencer_.previous.kpts_l,
                    sequencer_.current.kpts_l);

    sequencer_.current.T_r = pose_predictor_.estimatePoseFromFeatures(sequencer_.previous.kpts_l, 
                                                                      sequencer_.current.kpts_l, 
                                                                      stereo_.left().K_cv());

    if ( ! pose_predictor_.evaluateValidity(sequencer_.current.T_r, sequencer_.previous.T_r) )
      sequencer_.current.T_r = sequencer_.previous.T_r;

    sequencer_.current.T_r.translation() *= sequencer_.previous.scale;



    /***** Manage new features *****/
    detector_.bucketedFeatureDetection(sequencer_.current.img_l, 
                                        sequencer_.current.kpts_l);
    
    // std::vector<cv::KeyPoint> kpts_loop_candidate = sequencer_.current.kpts_l;

    // ROS_INFO_STREAM("detected - kpts_l.size(): " << sequencer_.current.kpts_l.size());

    std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> stereo_features = matcher_.circularMatching(sequencer_.current.img_l,
                                                                                                                sequencer_.current.img_r,
                                                                                                                sequencer_.previous.img_l,
                                                                                                                sequencer_.previous.img_r, 
                                                                                                                sequencer_.current.kpts_l, 
                                                                                                                sequencer_.current.kpts_r);

    sequencer_.storeFeatures(stereo_features.first, stereo_features.second);

    // ROS_INFO_STREAM("matched - match_left.size(): " << stereo_features.first.size() << ", match_right.size(): " << stereo_features.second.size());


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
                          "Stereo matches",
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


    T_kf_ *= sequencer_.current.T_r.matrix();
    LoopResult loop_result;
    is_keyframe_ = isKeyframe(T_kf_, 2, M_PI/2);
    if ( is_keyframe_ )
    {
      sequencer_.keyframes.push_back(sequencer_.current.img_l);

      // Check for loop closure
      std::vector<cv::KeyPoint> kpts_loop_candidate = sequencer_.current.kpts_l;
      cv::Mat desc_loop_candidate = detector_.computeDescriptor(sequencer_.current.img_l, 
                                                                kpts_loop_candidate);

      loop_result = loop_detector_.searchLoopCandidate(kpts_loop_candidate, 
                                                        desc_loop_candidate);
      
      // If loop - compute transformation
      if (loop_result.found)
      {
        std::vector<cv::KeyPoint> cur_kpts_matched, loop_kpts_matched;
        std::vector<cv::Point3f> landmarks_matched;
        std::vector<cv::DMatch> matches = matcher_.extractDescriptorMatches(desc_loop_candidate, loop_result.descriptors, 
                                                                            kpts_loop_candidate, loop_result.keypoints,
                                                                            sequencer_.current.world_points,
                                                                            cur_kpts_matched, loop_kpts_matched, landmarks_matched);

        Eigen::Affine3d T_loop = pose_predictor_.estimatePoseFromFeatures(cur_kpts_matched,
                                                                          loop_kpts_matched, 
                                                                          landmarks_matched,
                                                                          matches,
                                                                          stereo_.left().K_cv());
        T_loop.translation() *= 0;

        // displayWindowFeatures(sequencer_.keyframes[loop_result.match_id], loop_kpts_matched,
        //                       sequencer_.current.img_l, cur_kpts_matched,
        //                       "Loop features", 1920, 1080); 

        displayWindowMatches(sequencer_.keyframes[loop_result.match_id], kpts_loop_candidate, 
                              sequencer_.current.img_l, loop_result.keypoints,
                              matches,
                              "Loop matches", 1920, 1080);

        std::vector<cv::Point2f> loop_pts_matched;
        cv::KeyPoint::convert(loop_kpts_matched, loop_pts_matched);

        T_loop = motion_BA_.estimate(T_loop,
                                      landmarks_matched,
                                      loop_pts_matched);

        ROS_INFO_STREAM("Loop closure - T: \n" << T_loop.matrix());
      }

      toc_ = cv::getTickCount();
      printSummary(
        cam_left->header.stamp,
        (toc_ - tic_)/ cv::getTickFrequency(),
        T_kf_,
        sequencer_.current.world_points.size(),
        loop_result.found
      );

      T_kf_ = Eigen::Affine3d::Identity();
      keyframe_id_++;
    }

    // ROS_INFO_STREAM("VO - calculated pose: \n" << sequencer_.current.T_r.matrix());

    vo_pub_.publish( generateMsgInBody(cam_left->header.stamp,
                                        sequence_id_, 
                                        sequencer_.current.T_r,
                                        is_keyframe_,
                                        keyframe_id_,
                                        loop_result.found,
                                        loop_result.match_id,
                                        Eigen::Affine3d::Identity(),
                                        sequencer_.current.kpts_l,
                                        sequencer_.current.world_points,
                                        sequencer_.current.indices,
                                        frame_) );

    sequence_id_++;
    sequencer_.updatePreviousFrame();
    initialized_ = true;

  }

  // -----------------------------------------------------------------------------------------

  bool isKeyframe(Eigen::Affine3d& T_kf, double scale_thresh = 3, double rot_thresh = M_PI/9)
  {
    double scale = T_kf.translation().norm();
    Eigen::Vector3d euler = T_kf.linear().eulerAngles(0,1,2).cwiseAbs();

    if ( euler.x() > M_PI / 2 )
      euler.x() -= M_PI;

    if ( euler.y() > M_PI / 2 )
      euler.y() -= M_PI;

    if ( euler.z() > M_PI / 2 )
      euler.z() -= M_PI;

    // Keyframe criteria      
    if ( (scale > scale_thresh) 
      || (std::abs(euler.x()) > rot_thresh)  
      || (std::abs(euler.y()) > rot_thresh)  
      || (std::abs(euler.z()) > rot_thresh) )
    {
      // T_kf = Eigen::Affine3d::Identity();
      return true;
    }

    return false;
  }

};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_frontend");
	ROS_INFO("\n\n\n ----- Starting VO frontend ----- \n");

	VO vo;

  ros::spin();
}