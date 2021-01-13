// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

// Test using http://wiki.ros.org/video_stream_opencv
// - More specifically: rosrun image_view image_view image:=<image topic> [image transport type]

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/calib3d.hpp>

// #include <iostream>
#include <math.h>
// #include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <memory>
#include <time.h>
#include <algorithm> 
#include <numeric>
// #include <stdio.h>
// #include <stdlib.h>

#include "std_msgs/Float32.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud.h> 
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Transform.h>

#include "frame.h"
#include "feature_utils.h"
#include "pose_utils.h"
#include "point_correspondence.h"
// #include "motionBA/gtsam_pose_estimator.h"
#include "world_point.h"
#include "detector.h"

#include "opencv2/core/eigen.hpp"
#include <Eigen/Dense> 
// #include "sophus/se3.hpp"
#include <Eigen/Geometry> // Quaternions

#include <chrono>
#include <iomanip>

// #include <pcl/common/common_headers.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>

#include "vo/NorthEastHeading.h"


using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

// #define PI 3.14159265


class VisualOdometry
{

private:
  	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber image_sub_;
	// ros::Publisher pose_pub_;
	// ros::Publisher keypoint_pub_; 
	ros::Subscriber navi_sub;
	ros::Subscriber gps_sub;
	ros::Subscriber vel_sub;

	char* src;
	bool initialized_;
	bool isNewKeyframe_;
	int frameID_;

	// Relevant frames
	Frame previousFrame_;
	Frame previousKeyframe_;
	Frame currentFrame_;
	int numKpPrevKeyframe_;

	// Detector
    int GRID_SIZE;
	int MAX_FEATURES;					// For each grid

	// cv::Ptr<cv::GFTTDetector> extractor_;
	// cv::Ptr<cv::Feature2D> descriptor_;
	Detector detector_;

	// Non-Max supression 
    int NUM_RETURN_POINTS; 				// Number of returned points
    int TOLERANCE;

	// Tracker
	int TRACKED_THRESHOLD;
	std::vector<cv::Point2f> points2fPrev, points2fCurr;

	// Matcher
	float DISTANCE_THRESHOLD;
	cv::BFMatcher matcher_;

	// Camera
	cv::Mat K;					// float
	// cv::Mat Kd;				// double
	Eigen::Matrix3d Keig; 		// eigen
	cv::Mat distortion_;

	// Pose
	cv::Mat R_fl_b;
	cv::Mat R_fr_b;
    // cv::Mat R_c1c2, t_c1c2;
    cv::Mat R_wb, t_wb;
	cv::Mat euler_wb_;
	// double roll_,pitch_,yaw_;

	Eigen::Quaterniond q_wb;
	Sophus::SE3d SE3_prevKeyframe_;
	Sophus::SE3d SE3_relative_;
	double scale_;

	std::vector<cv::Point3f> worldPoints_;

	std::string VIDEO_;

	// GNSS stuff for scale
	// mA
	double dt_KF;
	double northPrevKeyframe_, eastPrevKeyframe_;
	double north_, east_;
	double theta_;
	
	// KITTI
	double latitudePrevKeyframe_, longitudePrevKeyframe_, altitudePrevKeyframe_;
	double latitude_, longitude_, altitude_;

	double absVelKeyframe_;

	bool first_gt_;
	double init_time_;

	double timestamp_;
	double x_gt,y_gt,z_gt, qx_gt,qy_gt,qz_gt,qw_gt;
	Eigen::Quaterniond q_gt;

	double timestamp_KF_;
	double timestamp_F_;
	double x_F,y_F,z_F, qx_F,qy_F,qz_F,qw_F;
	double x_KF,y_KF,z_KF, qx_KF,qy_KF,qz_KF,qw_KF;
	// Eigen::Quaterniond q_KF;
	
	cv::Mat t_b1b2_, euler_b1b2_;
	cv::Mat t_b1b2_gt_, euler_b1b2_gt_;

	cv::Mat euler_KF_;
	// double roll_KF,pitch_KF,yaw_KF;

	// Save results
	std::string resultPath_;
	std::string fileName_;
	std::string vo_results_name_;
	std::string gt_results_name_;

	std::string vo_results_euler_name_;
	std::string gt_results_euler_name_;

	std::string vo_results_euler_overall_name_;
	std::string gt_results_euler_overall_name_;

	std::string vo_scale_name_;

	// PoseEstimator::Ptr initial_pose_estimator;
	// GtsamPoseEstimator motionBA_(initial_pose_estimator, K);
	// PoseEstimate currentPose;

public:
	VisualOdometry(int GRID_SIZE=20, int MAX_FEATURES = 1000, 
				   int NUM_RETURN_POINTS = 1500, int TOLERANCE = 0.1, 
				   double DISTANCE_THRESHOLD = 0.7f,
				   int TRACKED_THRESHOLD = 500
		) : it_(nh_), 
		VIDEO_("kitti"), first_gt_(true), init_time_(0), timestamp_(0),
		initialized_(false), isNewKeyframe_(false), frameID_(0),
		GRID_SIZE(GRID_SIZE), MAX_FEATURES(MAX_FEATURES),
		NUM_RETURN_POINTS(NUM_RETURN_POINTS), TOLERANCE(TOLERANCE), DISTANCE_THRESHOLD(DISTANCE_THRESHOLD),
		TRACKED_THRESHOLD(TRACKED_THRESHOLD)
	{
		ROS_INFO_STREAM("Running: " << VIDEO_ << "\n");

		// Clear files
		// Overall with quaternion
		resultPath_ = ros::package::getPath("vo") + "/../../results/";
		vo_results_name_ = "stamped_traj_estimate.txt";
		gt_results_name_ = "stamped_groundtruth.txt";

		std::ofstream vo_results;
		vo_results.open(resultPath_ + vo_results_name_, std::ofstream::out | std::ofstream::trunc);
		vo_results.close();

		std::ofstream gt_results;
		gt_results.open(resultPath_ + gt_results_name_, std::ofstream::out | std::ofstream::trunc);
		gt_results.close();


		// Relative results
		vo_results_euler_name_ = "traj_estimate_euler_relative.txt";
		gt_results_euler_name_ = "groundtruth_euler_relative.txt";

		std::ofstream vo_results_euler;
		vo_results_euler.open(resultPath_ + vo_results_euler_name_, std::ofstream::out | std::ofstream::trunc);
		vo_results_euler.close();

		std::ofstream gt_results_euler;
		gt_results_euler.open(resultPath_ + gt_results_euler_name_, std::ofstream::out | std::ofstream::trunc);
		gt_results_euler.close();


		// Overall with euler
		vo_results_euler_overall_name_ = "traj_estimate_euler_overall.txt";
		gt_results_euler_overall_name_ = "groundtruth_euler_overall.txt";

		std::ofstream vo_results_euler_overall;
		vo_results_euler_overall.open(resultPath_ + vo_results_euler_overall_name_, std::ofstream::out | std::ofstream::trunc);
		vo_results_euler_overall.close();

		std::ofstream gt_results_euler_overall;
		gt_results_euler_overall.open(resultPath_ + gt_results_euler_overall_name_, std::ofstream::out | std::ofstream::trunc);
		gt_results_euler_overall.close();


		// resultPath_ = ros::package::getPath("backend") + "/../../results/";
		// fileName_ = "vo_poses.csv";
		// std::ofstream poseResults;
		// poseResults.open (resultPath_ + fileName_);

		// poseResults << "x,y,z,x_gt,y_gt,z_gt\n";
		// poseResults.close();


		// vo_scale_name_ = "vo_scale.csv";
		// std::ofstream scale_results;
		// scale_results.open (resultPath_ + vo_scale_name_);
		// scale_results << "scale\n";
		// scale_results.close();


		// https://stackoverflow.com/questions/44382267/how-to-find-the-focal-length-from-camera-matrix
		if (VIDEO_ == "ma2")
		{
			double params[9] = 
            {1832.966431,    0.000000, 1204.422219, 
                0.000000, 1829.857873, 1036.471360, 
                0.000000,    0.000000,    1.000000}; 
        	cv::Mat KdInit(3, 3, CV_64F, params);
			K = KdInit.clone();
		}

		if (VIDEO_ == "kitti") // https://github.com/uoip/monoVO-python
		{
			double params[9] = 
			{718.8560, 		  0, 607.1928, 
			 		0, 718.8560, 185.2157, 
			 		0, 		  0, 		1};

			// {9.799200e+02, 0.000000e+00, 6.900000e+02, 
			//  0.000000e+00, 9.741183e+02, 2.486443e+02, 
			//  0.000000e+00, 0.000000e+00, 1.000000e+00};

            // {9.842439e+02, 0.000000e+00, 6.900000e+02, 
			//  0.000000e+00, 9.808141e+02, 2.331966e+02, 
			//  0.000000e+00, 0.000000e+00, 1.000000e+00}; 
        	cv::Mat KdInit(3, 3, CV_64F, params);
			K = KdInit.clone();
		}
		cv::cv2eigen(K, Keig);

		distortion_ = (cv::Mat_<double>(5,1) << -0.3038870284329969, 0.09057337114251218, 6.702303710489991e-05, 0.000797130858026817, 0.0);

		t_wb = cv::Mat::zeros(3,1, CV_64F);

		R_wb = cv::Mat::eye(3,3, CV_64F);

		
		cv::Mat euler_w_cl = (cv::Mat_<double>(3,1) << 0, 
													 0, 
													 -1.25663706144);
		cv::Rodrigues(euler_w_cl, R_fl_b);

		cv::Mat euler_w_cr = (cv::Mat_<double>(3,1) << 0, 
													 0, 
													 1.25663706144);
		cv::Rodrigues(euler_w_cr, R_fr_b);

		// cv::Mat initRot = (cv::Mat_<double>(3,1) <<  
		// 					0.08109187, 
		// 					0.02287384, 
		// 					0.53);
		// cv::Rodrigues(initRot, R_wb);

		// dt_KF = 7.8 / double(77);

		x_KF=0; y_KF=0; z_KF=0;

		matcher_ = cv::BFMatcher(detector_.getDefaultNorm());

		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("image", 1, &VisualOdometry::imageCb, this);
		if (VIDEO_ == "ma2")
			// navi_sub = nh_.subscribe("/navigation/eta", 1, &VisualOdometry::readGNSS, this);
			navi_sub = nh_.subscribe("/tf", 1, &VisualOdometry::readKittiTf, this);

		if (VIDEO_ == "kitti")
		{
			// gps_sub = nh_.subscribe("/kitti/oxts/gps/fix", 1, &VisualOdometry::readNavSat, this);
			// vel_sub = nh_.subscribe("/kitti/oxts/gps/vel", 1, &VisualOdometry::readVel, this);
			navi_sub = nh_.subscribe("/tf", 1, &VisualOdometry::readKittiTf, this);
		}
		// pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("vo_pose", 1000);
		// keypoint_pub_ = nh_.advertise<sensor_msgs::PointCloud>("vo_keypoints", 1000);

		// detect_pub_ = nh_.advertise<std_msgs::Float32>("path_angle", 1000);


		// cv::namedWindow(OPENCV_WINDOW, WINDOW_NORMAL);

	}

	~VisualOdometry()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	


	void readKittiTf(const tf2_msgs::TFMessage& msg)
	{
		timestamp_ = msg.transforms[0].header.stamp.sec + (msg.transforms[0].header.stamp.nsec / 1e9);
		if (first_gt_)
		{
			init_time_ = timestamp_;
			first_gt_ = false;
		}
		timestamp_ -= init_time_;
		// ROS_INFO_STREAM("timestamp_: " << timestamp_);

		x_gt = msg.transforms[0].transform.translation.x;
		y_gt = msg.transforms[0].transform.translation.y;
		z_gt = msg.transforms[0].transform.translation.z;

		qx_gt = msg.transforms[0].transform.rotation.x;
		qy_gt = msg.transforms[0].transform.rotation.y;
		qz_gt = msg.transforms[0].transform.rotation.z;
		qw_gt = msg.transforms[0].transform.rotation.w;

		q_gt.x() = qx_gt;
		q_gt.y() = qy_gt;
		q_gt.z() = qz_gt;
		q_gt.w() = qw_gt;    
	}

	void readGNSS(const vo::NorthEastHeading& nav_msg)
	{
		north_ = nav_msg.north;
		east_  = nav_msg.east; 		// - nav_msg.east;
		theta_ = nav_msg.heading; 	// - nav_msg.heading
	}

	void readNavSat(const sensor_msgs::NavSatFix& nav_msg)
	{
		longitude_ = nav_msg.longitude;
		latitude_ = nav_msg.latitude;
		altitude_ = nav_msg.altitude;
	}

	void readVel(const geometry_msgs::TwistStamped& vel_msg)
	{
		absVelKeyframe_ = sqrt( pow(vel_msg.twist.linear.x, 2) + pow(vel_msg.twist.linear.y, 2) + pow(vel_msg.twist.linear.z, 2) );
	}


	void displayWindow(cv::Mat image1, cv::Mat image2, std::string name, int resizeWidth, int resizeHeight, int key)
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



	void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		}
		catch (cv_bridge::Exception& e)
		{

			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	
		setNewFramePos(); // Register new frame

		int x = 215; 
		int y = 0;
		int width = 1440;  // cv_ptr->image.cols / 1.7; 
		int height = 1172; // cv_ptr->image.rows / 1.75; 
		
		int cols = cv_ptr->image.cols;
		int rows = cv_ptr->image.rows;

		cv::Mat img;
		cv::Mat undist_img;
		cv::Mat K_undist;
		if (VIDEO_ == "ma2")
		{
			// img = cv_ptr->image(cv::Range(0, (cv_ptr->image.rows / 2)), cv::Range(0, cv_ptr->image.rows - (rows % GRID_SIZE))); 			// Bag 4, F		
			// img = cv_ptr->image(cv::Rect(x, y, width - ( (cols - width) % GRID_SIZE), height - ( (rows - height) % GRID_SIZE))); 		// Bag 5, F
			// img = cv_ptr->image( cv::Rect(0, 0, cols - (cols % GRID_SIZE), rows - (rows % GRID_SIZE)) );									// Bag 5, F IR
			// cv::undistort(cv_ptr->image, undist_img, K, distortion_, K_undist);
			// img = undist_img(cv::Rect(x, y, width, height));											 							// Bag 5, FL
			img = cv_ptr->image(cv::Rect(x, y, width, height));											 							// Bag 5, FL
		}
		else
		{
			img = cv_ptr->image( cv::Rect(0, 0, cols - (cols % GRID_SIZE), rows - (rows % GRID_SIZE)) );
		}

		
		// ROS_INFO_STREAM("MOD: img.cols: " << img.cols % GRID_SIZE << " img.rows: " << img.rows % GRID_SIZE);
		// displayWindow(img, cv::Mat(), "Original image", 1000, 500, 3);


		currentFrame_.image = img; 
		currentFrame_.descriptor = cv::Mat(); 
		currentFrame_.keypoints.clear(); 

		if (previousFrame_.keypoints.empty())
		{	
			detector_.gridDetectAndCompute(currentFrame_);
			// detector_.detect(currentFrame_);
			// detector_.compute(currentFrame_);
			previousKeyframe_ = currentFrame_;

			
		
			Eigen::Matrix3d R_init_eig = q_gt.normalized().toRotationMatrix();
			eigen2cv(R_init_eig, R_wb);

			t_wb = (cv::Mat_<double>(3,1) << x_gt, 
											 y_gt, 
											 z_gt);

			// if (VIDEO_ == "ma2")
			// {
			// 	cv::Mat t_wc = (cv::Mat_<double>(3,1) << 0.0618034, 
			// 								 -0.1902113, 
			// 								 -2.68);
        	
			// 	cv::Mat initEuler = (cv::Mat_<double>(3,1) << 0, 
			// 												0, 
			// 												-1.25663706144);
			// 	cv::Mat initRot;
			// 	cv::Rodrigues(initEuler, initRot);
			// }
			


			numKpPrevKeyframe_ = previousKeyframe_.keypoints.size();

			setNewKeyframe();
			// poseToCsv(resultPath_ + fileName_); 
			// poseToTxt();

			ROS_INFO("First frame processed");
			ROS_INFO_STREAM("Number of keypoints for keyframe: " << numKpPrevKeyframe_);
		}


		else
		{
			frameID_++;
			
			if (previousFrame_.keypoints.size() < std::max(TRACKED_THRESHOLD, int(numKpPrevKeyframe_ * 0.9)) || !(frameID_%20) )
			{
				detector_.gridDetectAndCompute(currentFrame_);
				// detector_.detect(currentFrame_);
				// detector_.compute(currentFrame_);

				numKpPrevKeyframe_ = currentFrame_.keypoints.size();
				
				ROS_INFO_STREAM("New detect - Number of keypoints for keyframe: " << numKpPrevKeyframe_);
				
				isNewKeyframe_ = true;
			}
			else
			{
				points2fPrev.clear(); points2fCurr.clear();
				track(previousFrame_, currentFrame_);
				// track(previousFrame_, currentFrame_, points2fPrev, points2fCurr);
				detector_.compute(currentFrame_);

				ROS_INFO_STREAM("Tracked number of features: " << currentFrame_.keypoints.size());
			}
			
			
			cv::Mat imgKps;
    		cv::drawKeypoints(currentFrame_.image, currentFrame_.keypoints, imgKps, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
			displayWindow(imgKps, cv::Mat(), "Features", imgKps.cols/2, imgKps.rows/2, 3);


			scale_ = getScale(VIDEO_);
			// ROS_INFO_STREAM("scale_" << scale_);

			if (scale_) // && (previousFrame_.keypoints.size() < numKpPrevKeyframe_ * 0.9) ) // Decide if new keyframe
			{
				// ROS_INFO("--- New KF ---");			

				///// MATCH /////
				std::vector<cv::Point2f> points2fPrev, points2fCurr;
				std::vector<int> ids; 
				std::vector<std::vector<cv::DMatch>> matches;
				matcher_.knnMatch(previousKeyframe_.descriptor, currentFrame_.descriptor, matches, 2);

				// Only keep matched points 
				std::vector<cv::DMatch> good_matches = extractGoodRatioMatches(matches, DISTANCE_THRESHOLD);
				extractMatchingPoints(good_matches, previousKeyframe_.keypoints, currentFrame_.keypoints, points2fPrev, points2fCurr);
		

				///// ESTIMATE POSE /////
				cv::Mat E, F, inliers; 
				
				// ROS_INFO_STREAM("points2fCurr.size(): " << points2fCurr.size());

				// https://docs.opencv.org/master/d9/d0c/group__calib3d.html
				if (points2fCurr.size() >= 8) // 5 for Essential, 8 for fundamental
				{
					E = cv::findEssentialMat(points2fPrev, points2fCurr, K, cv::RANSAC, 0.999, 1.0, inliers); 
					F = cv::findFundamentalMat(points2fPrev, points2fCurr, cv::RANSAC, 0.999, 1.0); 						
					
					removeRANSACoutliers(good_matches, inliers, points2fPrev, points2fCurr);	// Track + match
					// removeRANSACoutliers(inliers, points2fPrev, points2fCurr);				// Track only
					// ROS_INFO_STREAM("Num mathces after ratio test and RANSAC: " << good_matches.size() );

					// std::vector<cv::Point2f> betterPoints2fPrev, betterPoints2fCurr;
					// correctMatches(F, points2fPrev, points2fCurr, betterPoints2fPrev, betterPoints2fCurr); // https://docs.opencv.org/master/d9/d0c/group__calib3d.html#gac42edda3a3a0f717979589fcd6ac0035
					// points2fPrev = betterPoints2fPrev; 
					// points2fCurr = betterPoints2fCurr;

				    cv::Mat R_c1c2, t_c1c2;
					cv::recoverPose(E, points2fPrev, points2fCurr, K, R_c1c2, t_c1c2); // z = viewer direction, x and y follows camera frame
										
					// cv::Mat imgKps = drawKpts(currentFrame_.image, points2fCurr);
					// displayWindow(imgKps, cv::Mat(), "Features", imgKps.cols/2, imgKps.rows/2, 3);

					// /***** DRAW MATCHES *****/
					// cv::Mat im_match; 
					// cv::drawMatches(previousKeyframe_.image, previousKeyframe_.keypoints, currentFrame_.image, currentFrame_.keypoints, good_matches, im_match); 
					// displayWindow(im_match, cv::Mat(), "Matches", 2000, 500, 3);

					/***** VISUALIZE EPIPOLAR LINES *****/ https://docs.opencv.org/3.4/da/de9/tutorial_py_epipolar_geometry.html
					cv::Mat imageEpiLeft = drawKpts(previousKeyframe_.image.clone(), points2fPrev);
					imageEpiLeft = drawEpiLines(imageEpiLeft, F, 2, points2fCurr);
					cv::Mat imageEpiRight = drawKpts(currentFrame_.image.clone(), points2fCurr);
					drawEpiLines(imageEpiRight, F, 1, points2fPrev);
					
					displayWindow(imageEpiLeft, imageEpiRight, "Epilines", 2000, 500, 3);



					cv::Mat R_b1b2, t_b1b2;
					if (VIDEO_ == "kitti")
						cv2body_kitti(R_c1c2, t_c1c2, R_b1b2, t_b1b2);
					if (VIDEO_ == "ma2")
					{
						// cv2body_ma2(R_c1c2, t_c1c2, R_b1b2, t_b1b2);
						cv2body_ma2_FL(R_c1c2, t_c1c2, R_b1b2, t_b1b2, R_fl_b);
						// cv2body_ma2_FR(R_c1c2, t_c1c2, R_b1b2, t_b1b2, R_fr_b);
						// R_b1b2 = R_c1c2;
						// t_b1b2 = t_c1c2;
					}

					// cv::Mat euler_c1c2;
					// cv::Rodrigues(R_c1c2, euler_c1c2);
					// cv::Mat euler_b1b2;
					// cv::Rodrigues(R_b1b2, euler_b1b2);
					// ROS_INFO_STREAM("c1c2 rotation: " << euler_c1c2);
					// ROS_INFO_STREAM("b1b2 rotation: " << euler_b1b2);
					// ROS_INFO_STREAM("c1c2 translation: " << t_c1c2);
					// ROS_INFO_STREAM("b1b2 translation: " << t_b1b2 << "\n\n");


					// correctInvalidRotation(R_b1b2);

					bool valid_transformation = false;
					if (VIDEO_ == "kitti")
						valid_transformation = (isValidRotation(R_b1b2, M_PI/3) && isValidTranslation(t_b1b2, 0.7) );
					if (VIDEO_ == "ma2")
						// valid_transformation = (isValidRotation_cam(R_c1c2, M_PI/2) && isValidTranslation_cam_fl(t_c1c2, 0.9));
						// valid_transformation = (isValidRotation_cam(R_c1c2, M_PI/2) && isValidTranslation_cam_fr(t_c1c2, 0.9));
						valid_transformation = isValidTranslation_cam_fl(t_c1c2, 0.9);
						// valid_transformation = isValidTranslation_cam_fr(t_c1c2, 0.9);

					// correctInvalidRotation(R_b1b2);
					R_b1b2 = correctInvalidRotation(R_b1b2);

					if (valid_transformation)
					{
						if (VIDEO_ == "ma2")
						{
							// correctInvalidRotation(R_b1b2);
							// correctInvalidTranslation(t_b1b2, 0.8);


							cv::Mat euler_wb_prev;
							cv::Mat R_wb_prev = R_wb;
							cv::Rodrigues(R_wb_prev, euler_wb_prev);
							
							cv::Mat R_wb_cur;
							cv::Mat euler_wb_cur;
							Eigen::Matrix3d R_init_eig = q_gt.normalized().toRotationMatrix();
							eigen2cv(R_init_eig, R_wb_cur); 
							cv::Rodrigues(R_wb_cur, euler_wb_cur);

							cv::Mat euler_wb_diff = euler_wb_cur - euler_wb_prev;
							
							cv::Mat euler_b1b2;
							cv::Rodrigues(R_b1b2, euler_b1b2);

							euler_wb_diff = (0.1*euler_wb_diff + 0.9*euler_b1b2);

							// cv::Mat R_wb_diff;
							cv::Rodrigues(euler_wb_diff, R_b1b2);

							t_wb = t_wb + R_wb * t_b1b2 * scale_; 
							R_wb = R_wb * R_b1b2; 
						}
						
						if (VIDEO_ == "kitti")
						{
							t_wb = t_wb + R_wb * t_b1b2 * scale_; 
							R_wb = R_wb * R_b1b2;  
						}

				
						// Eigen::Matrix3d R_init_eig = q_gt.normalized().toRotationMatrix();
						// eigen2cv(R_init_eig, R_wb); 
						// // t_wb = (cv::Mat_<double>(3,1) << x_gt, 
						// // 								y_gt, 
						// // 								z_gt);
						// t_wb = t_wb + R_wb * t_b1b2 * scale_; 
						// R_wb = R_wb * R_b1b2;  

						// Relative
						t_b1b2_ = R_wb * t_b1b2 * scale_;
						cv::Rodrigues(R_b1b2, euler_b1b2_);	
						
						// Overall
						cv::Rodrigues(R_wb, euler_wb_);			// Euler
						Eigen::Matrix3d R_wb_eig;
						cv::cv2eigen(R_wb, R_wb_eig);
						Eigen::Quaterniond EigenQuat(R_wb_eig);
						q_wb = EigenQuat;						// Quaternion
						// Eigen::Matrix3d R_init_eig = q_wb.normalized().toRotationMatrix();


						// cv::Mat euler_c1c2;
						// cv::Rodrigues(R_c1c2, euler_c1c2);
						// cv::Mat euler_b1b2;
						// cv::Rodrigues(R_b1b2, euler_b1b2);
						// ROS_INFO_STREAM("c1c2 rotation: " << euler_c1c2);
						// ROS_INFO_STREAM("b1b2 rotation: " << euler_b1b2);
						// ROS_INFO_STREAM("c1c2 translation: " << t_c1c2);
						// ROS_INFO_STREAM("b1b2 translation: " << t_b1b2 << "\n\n");

						setNewKeyframe();

						poseToTxt(); 
						poseRelativeToTxt();
						poseToTxtEuler();

						// poseToCsv(resultPath_ + fileName_); 
						// scaleToCSV();
					}
				}
				else
				{
					ROS_INFO("Not enough good matches");
				}
				

				///// TRIANGULATE /////
				// https://docs.opencv.org/3.1.0/d9/d0c/group__calib3d.html#gad3fc9a0c82b08df034234979960b778c
				// https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#triangulatepoints
				// if (good_matches.size() >= 8) 
				// {
				// 	// cv::Mat Rt0 = cv::Mat::eye(3, 4, CV_64FC1);
				// 	// cv::Mat Rt1 = cv::Mat::eye(3, 4, CV_64FC1);
				// 	// R_c1c2.copyTo(Rt1.rowRange(0,3).colRange(0,3));
				// 	// t_c1c2.copyTo(Rt1.rowRange(0,3).col(3));
					
				// 	int N = points2fPrev.size();
				// 	cv::Mat point3d_homo(4, N, CV_64FC1);                                               // https://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints
				// 	cv::triangulatePoints( K*Rt0, K*Rt1, points2fPrev, points2fCurr, point3d_homo);   // https://gist.github.com/cashiwamochi/8ac3f8bab9bf00e247a01f63075fedeb


				// 	// std::vector<cv::Point3f> worldPoints;
				// 	for(int i = 0; i < point3d_homo.cols; i++) {
				// 		cv::Point3f wp;
				// 		cv::Mat p3d;
				// 		cv::Mat _p3h = point3d_homo.col(i);
				// 		cv::convertPointsFromHomogeneous(_p3h.t(), p3d); // https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#gac42edda3a3a0f717979589fcd6ac0035

				// 		wp.x = p3d.at<float>(0);
				// 		wp.y = p3d.at<float>(1);
				// 		wp.z = p3d.at<float>(2);

				// 		worldPoints_.push_back(wp);
				// 		// ROS_INFO_STREAM("wp.z: " << wp.z);

				// 	}
				// }

				// ROS_INFO_STREAM("3Dpts.size(): " << worldPoints_.size() << "\n3Dpts: " << worldPoints_);

				
				// setNewKeyframe();
			}						
			
		}

		previousFrame_ = currentFrame_; 
		
	
	}

	void setNewFramePos()
	{
		timestamp_F_ = timestamp_;
		x_F  = x_gt;
		y_F  = y_gt;
		z_F  = z_gt;
		qx_F = qx_gt;
		qy_F = qy_gt;
		qz_F = qz_gt;
		qw_F = qw_gt;
	}


	void setNewKeyframe()
	{
		Eigen::Quaterniond q_KF_prev;
		q_KF_prev.x() = qx_KF;
		q_KF_prev.y() = qy_KF;
		q_KF_prev.z() = qz_KF;
		q_KF_prev.w() = qw_KF;    

		cv::Mat R_prev, euler_prev;
		Eigen::Matrix3d R_prev_eig = q_KF_prev.normalized().toRotationMatrix();
		eigen2cv(R_prev_eig, R_prev); 
		cv::Rodrigues(R_prev, euler_prev);

		cv::Mat t_prev = (cv::Mat_<double>(3,1) << x_KF, 
													y_KF, 
													z_KF);


		timestamp_KF_ = timestamp_F_;
		x_KF  = x_F;
		y_KF  = y_F;
		z_KF  = z_F;
		qx_KF = qx_F;
		qy_KF = qy_F;
		qz_KF = qz_F;
		qw_KF = qw_F;



		Eigen::Quaterniond q_KF_cur;
		q_KF_cur.x() = qx_KF;
		q_KF_cur.y() = qy_KF;
		q_KF_cur.z() = qz_KF;
		q_KF_cur.w() = qw_KF;    

		cv::Mat R_cur, euler_cur;
		Eigen::Matrix3d R_cur_eig = q_KF_cur.normalized().toRotationMatrix();
		eigen2cv(R_cur_eig, R_cur); 
		cv::Rodrigues(R_cur, euler_cur);
		euler_KF_ = euler_cur;

		cv::Mat t_cur = (cv::Mat_<double>(3,1) << x_KF, 
													y_KF, 
													z_KF);

		t_b1b2_gt_ = t_cur - t_prev;
		euler_b1b2_gt_ = euler_cur - euler_prev;					

		northPrevKeyframe_ = north_;
		eastPrevKeyframe_ = east_;

		numKpPrevKeyframe_ = currentFrame_.keypoints.size();
		previousKeyframe_ = currentFrame_;

		isNewKeyframe_ = false;
	}

	double getScale(std::string videoType)
	{
		return sqrt( pow(x_gt - x_KF, 2) + pow(y_gt - y_KF, 2) + pow(z_gt - z_KF, 2) );

		// if (VIDEO_ == "ma2")
		// 	// return sqrt( pow(north_ - northPrevKeyframe_, 2) + pow(east_ - eastPrevKeyframe_, 2) );
		// 	return sqrt( pow(x_gt - x_KF, 2) + pow(y_gt - y_KF, 2) + pow(z_gt - z_KF, 2) );
		// else if (VIDEO_ == "kitti")
		// {
		// 	// scale_ = absVelKeyframe_ * dt_KF;
		// 	return sqrt( pow(x_gt - x_KF, 2) + pow(y_gt - y_KF, 2) + pow(z_gt - z_KF, 2) );
		// 	// scale_ = estimateScale(points2fPrev, points2fCurr, Keig) / 1000;
		// }
		// else
		// 	return 1;
	}

	void poseToCsv(std::string filepath)
    {
		std::ofstream poseResults;
		poseResults.open (filepath, std::ios_base::app);
		poseResults << t_wb.at<double>(0) 	  << "," 	// x vo
					<< t_wb.at<double>(1) 	  << "," 	// y vo
					<< t_wb.at<double>(2) 	  << "," 	// z vo
					<< x_gt << "," 					// x ground truth
					<< y_gt << "," 					// y ground truth
					<< z_gt << "\n"; 					// z ground truth
		poseResults.close();

		// if (VIDEO_ == "ma2")
		// {
		// 	std::ofstream poseResults;
		// 	poseResults.open (filepath, std::ios_base::app);
		// 	poseResults << t_wb.at<double>(0) 	  << "," 	// x vo
		// 				<< t_wb.at<double>(1) 	  << "," 	// y vo
		// 				<< t_wb.at<double>(2) 	  << "," 	// z vo
		// 				<< northPrevKeyframe_ << "," 	// N
		// 				<< eastPrevKeyframe_  << "\n"; 	// E      
		// 	poseResults.close();
		// }
		// else
		// {
		// 	std::ofstream poseResults;
		// 	poseResults.open (filepath, std::ios_base::app);
		// 	poseResults << t_wb.at<double>(0) 	  << "," 	// x vo
		// 				<< t_wb.at<double>(1) 	  << "," 	// y vo
		// 				<< t_wb.at<double>(2) 	  << "," 	// z vo
		// 				<< x_gt << "," 					// x ground truth
		// 				<< y_gt << "," 					// y ground truth
		// 				<< z_gt << "\n"; 					// z ground truth
		// 	poseResults.close();
		// }
    }

	void scaleToCSV()
	{
		std::ofstream scale_results;
		scale_results.open (resultPath_ + vo_scale_name_, std::ios_base::app);
		scale_results << scale_ << "\n";
		scale_results.close();
	}

	void poseRelativeToTxt() // https://github.com/uzh-rpg/rpg_trajectory_evaluation
    {
		// Ground truth
		std::ofstream gt_results;
		gt_results.open (resultPath_ + gt_results_euler_name_, std::ios_base::app);
		if (VIDEO_ == "ma2")
		{
			gt_results  << timestamp_KF_ << " " 	// timestamp for trajectory
						<< t_b1b2_gt_.at<double>(0)			 << " "		// x translation 
						<< t_b1b2_gt_.at<double>(1)			 << " "		// y translation 
						<< t_b1b2_gt_.at<double>(2) 		 << " "		// z translation
						<< euler_b1b2_gt_.at<double>(0)		 		<< " "		// x quaternion
						<< euler_b1b2_gt_.at<double>(1)		 		<< " "		// y quaternion
						<< euler_b1b2_gt_.at<double>(2) 		 	<< "\n";	// w quaternion
		}
		if (VIDEO_ == "kitti")
		{
			gt_results  << timestamp_KF_ << " " 	// timestamp for trajectory
						<< t_b1b2_gt_.at<double>(0)			 << " "		// x translation 
						<< t_b1b2_gt_.at<double>(2) 		 << " "		// z translation at y pos (switched)
						<< t_b1b2_gt_.at<double>(1) 		 << " "		// y translation at z pos (switched)
						<< euler_b1b2_gt_.at<double>(0)		 		<< " "		// x quaternion
						<< euler_b1b2_gt_.at<double>(1)		 		<< " "		// y quaternion
						<< euler_b1b2_gt_.at<double>(2) 		 	<< "\n";	// w quaternion
		}
		
		gt_results.close();

		// VO
		std::ofstream vo_results;
		vo_results.open (resultPath_ + vo_results_euler_name_, std::ios_base::app);
		if (VIDEO_ == "ma2")
		{
			vo_results  << timestamp_KF_ 		<< " " 		// timestamp for trajectory
						<< t_b1b2_.at<double>(0)	<< " "		// x translation 
						<< t_b1b2_.at<double>(1) 	<< " "		// y translation
						<< t_b1b2_.at<double>(2)	<< " "		// y translation at z pos (switched)
						<< euler_b1b2_.at<double>(0)		 		<< " "		// x quaternion
						<< euler_b1b2_.at<double>(1)		 		<< " "		// y quaternion
						<< euler_b1b2_.at<double>(2) 		 	<< "\n";	// w quaternion
		}
		if (VIDEO_ == "kitti")
		{
			vo_results  << timestamp_KF_ 		<< " " 		// timestamp for trajectory
						<< t_b1b2_.at<double>(0)	<< " "		// x translation 
						<< t_b1b2_.at<double>(2) 	<< " "		// z translation at y pos (switched) - KITTI
						<< t_b1b2_.at<double>(1) 	<< " "		// y translation at z pos (switched) - KITTI
						// << 0	<< " "		// y translation at z pos (switched)
						<< euler_b1b2_.at<double>(0)		 		<< " "		// x quaternion
						<< euler_b1b2_.at<double>(1)		 		<< " "		// y quaternion
						<< euler_b1b2_.at<double>(2) 		 	<< "\n";	// w quaternion
		}
		vo_results.close();
    }


	void poseToTxtEuler() // https://github.com/uzh-rpg/rpg_trajectory_evaluation
    {
		// Ground truth
		std::ofstream gt_results;
		gt_results.open (resultPath_ + gt_results_euler_overall_name_, std::ios_base::app);
		if (VIDEO_ == "ma2")
		{
			gt_results  << timestamp_KF_ << " " 	// timestamp for trajectory
						<< x_KF			 << " "		// x translation 
						<< y_KF			 << " "		// y translation
						<< z_KF 		 << " "		// z translation 
						<< euler_KF_.at<double>(0)		 << " "		// roll
						<< euler_KF_.at<double>(1)		 << " "		// pitch
						<< euler_KF_.at<double>(2)		 << "\n";		// yaw
		}

		if (VIDEO_ == "kitti")
		{
			gt_results  << timestamp_KF_ << " " 	// timestamp for trajectory
						<< x_KF			 << " "		// x translation 
						<< z_KF 		 << " "		// z translation at y pos (switched)
						<< y_KF			 << " "		// y translation at z pos (switched)
						<< euler_KF_.at<double>(0)		 << " "		// roll
						<< euler_KF_.at<double>(1)		 << " "		// pitch
						<< euler_KF_.at<double>(2)		 << "\n";		// yaw
		}
		gt_results.close();

		// VO 
		std::ofstream vo_results;
		vo_results.open (resultPath_ + vo_results_euler_overall_name_, std::ios_base::app);
		if (VIDEO_ == "ma2")
		{
			vo_results  << timestamp_KF_ 		<< " " 		// timestamp for trajectory
						<< t_wb.at<double>(0)	<< " "		// x translation 
						<< t_wb.at<double>(1) 	<< " "		// y translation
						<< t_wb.at<double>(2)	<< " "		// y translation at z pos (switched)
						// << t_wb.at<double>(1)	<< " "		// y translation at z pos (switched)
						<< euler_wb_.at<double>(0)		 << " "		// roll
						<< euler_wb_.at<double>(1)		 << " "		// pitch
						<< euler_wb_.at<double>(2)		 << "\n";		// yaw
		}
		if (VIDEO_ == "kitti")
		{
			vo_results  << timestamp_KF_ 		<< " " 		// timestamp for trajectory
						<< t_wb.at<double>(0)	<< " "		// x translation 
						<< t_wb.at<double>(2) 	<< " "		// z translation at y pos (switched) - KITTI
						<< 0	<< " "		// y translation at z pos (switched)
						// << t_wb.at<double>(1)	<< " "		// y translation at z pos (switched)
						<< euler_wb_.at<double>(0)		 << " "		// roll
						<< euler_wb_.at<double>(1)		 << " "		// pitch
						<< euler_wb_.at<double>(2)		 << "\n";		// yaw
		}
		vo_results.close();
    }


	void poseToTxt() // https://github.com/uzh-rpg/rpg_trajectory_evaluation
    {
		// Ground truth
		std::ofstream gt_results;
		gt_results.open (resultPath_ + gt_results_name_, std::ios_base::app);
		if (VIDEO_ == "ma2")
		{
			gt_results  << timestamp_KF_ << " " 	// timestamp for trajectory
						<< x_KF			 << " "		// x translation 
						<< y_KF			 << " "		// y translation
						<< z_KF 		 << " "		// z translation 
						<< qx_KF		 << " "		// x quaternion
						<< qy_KF		 << " "		// y quaternion
						<< qz_KF		 << " "		// y quaternion
						<< qw_KF 		 << "\n";	// w quaternion
		}

		if (VIDEO_ == "kitti")
		{
			gt_results  << timestamp_KF_ << " " 	// timestamp for trajectory
						<< x_KF			 << " "		// x translation 
						<< z_KF 		 << " "		// z translation at y pos (switched)
						<< y_KF			 << " "		// y translation at z pos (switched)
						<< qx_KF		 << " "		// x quaternion
						<< qy_KF		 << " "		// y quaternion
						<< qz_KF		 << " "		// y quaternion
						<< qw_KF 		 << "\n";	// w quaternion
		}
		gt_results.close();

		// VO
		std::ofstream vo_results;
		vo_results.open (resultPath_ + vo_results_name_, std::ios_base::app);
		if (VIDEO_ == "ma2")
		{
			vo_results  << timestamp_KF_ 		<< " " 		// timestamp for trajectory
						<< t_wb.at<double>(0)	<< " "		// x translation 
						<< t_wb.at<double>(1) 	<< " "		// y translation
						<< t_wb.at<double>(2)	<< " "		// y translation at z pos (switched)
						// << t_wb.at<double>(1)	<< " "		// y translation at z pos (switched)
						<< q_wb.x()		 		<< " "		// x quaternion
						<< q_wb.y()		 		<< " "		// y quaternion
						<< q_wb.z()		 		<< " "		// y quaternion
						<< q_wb.w() 		 	<< "\n";	// w quaternion
		}
		if (VIDEO_ == "kitti")
		{
			vo_results  << timestamp_KF_ 		<< " " 		// timestamp for trajectory
						<< t_wb.at<double>(0)	<< " "		// x translation 
						<< t_wb.at<double>(2) 	<< " "		// z translation at y pos (switched) - KITTI
						<< 0	<< " "		// y translation at z pos (switched)
						// << t_wb.at<double>(1)	<< " "		// y translation at z pos (switched)
						<< q_wb.x()		 		<< " "		// x quaternion
						<< q_wb.y()		 		<< " "		// y quaternion
						<< q_wb.z()		 		<< " "		// y quaternion
						<< q_wb.w() 		 	<< "\n";	// w quaternion
		}
		vo_results.close();
    }


};
	

int main(int argc, char **argv)
{
	// if (argc != 7)
	// {
	// 	ROS_INFO_STREAM(argc); 
	// 	ROS_ERROR("Not properly formed args"); 
	// 	return -1; 
	// }

	ros::init(argc, argv, "vo");

	ROS_INFO("\n\n\n ----- Starting VO module ----- \n");
	// VisualOdometry ic();
	
	VisualOdometry ic;
	ros::spin();
	return 0;
}





