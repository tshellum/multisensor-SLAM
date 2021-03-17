#pragma once

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <Eigen/Dense> 

#include "opencv2/core/eigen.hpp"
#include <opencv2/calib3d.hpp>


class PinholeModel
{
private:    
    std::string name_;

	cv::Mat previous_image_;
	cv::Mat current_image_;

    // Projection parameters
    double _fx, _fy, _cx, _cy;
    int _width, _height;
    
    // Distortion parameters
    double _k1, _k2, _p1, _p2, _k3;
    cv::Mat _distortion;
    
    // Calibration matrix
    cv::Mat _K_cv;
   	Eigen::Matrix3d _K_eig; 	

public:
    PinholeModel(ros::NodeHandle nh, const std::string name) 
	: name_(name), 
	  _k3(0.0),
	  previous_image_(cv::Mat()), current_image_(cv::Mat())
    {
        nh.getParam("/"+name_+"/image_width", _width);
        nh.getParam("/"+name_+"/image_height", _height);
        nh.getParam("/"+name_+"/projection_parameters/fx", _fx);
        nh.getParam("/"+name_+"/projection_parameters/fy", _fy);
        nh.getParam("/"+name_+"/projection_parameters/cx", _cx);
        nh.getParam("/"+name_+"/projection_parameters/cy", _cy);
        nh.getParam("/"+name_+"/distortion_parameters/k1", _k1);
        nh.getParam("/"+name_+"/distortion_parameters/k2", _k2);
        nh.getParam("/"+name_+"/distortion_parameters/p1", _p1);
        nh.getParam("/"+name_+"/distortion_parameters/p2", _p2);

        _K_cv = (cv::Mat_<double>(3,3) << 
                _fx,   0, _cx,
                  0, _fy, _cy,
                  0,   0,   1);
        
        cv::cv2eigen(_K_cv, _K_eig);

        _distortion = (cv::Mat_<double>(5,1) << _k1, _k2, _p1, _p2, _k3);
    };

    ~PinholeModel() {};

    cv::Mat K_cv() {return _K_cv;};
    Eigen::Matrix3d K_eig() {return _K_eig;};
    cv::Mat distortion() {return _distortion;};

    int getWidth()  {return _width;};
    int getHeight() {return _height;};
    cv::Mat getPreviousImage() {return previous_image_;};
    cv::Mat getCurrentImage()  {return current_image_;};

	cv::Mat storeGray(const sensor_msgs::ImageConstPtr &img_msg);
    void rectify(cv::Mat& img, cv::Mat K_undist);
    void crop(cv::Mat& img, int x1, int x2, int y1, int y2);
};


cv::Mat PinholeModel::storeGray(const sensor_msgs::ImageConstPtr &img_msg)
{
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr  = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	if (! current_image_.empty())
		previous_image_ = current_image_;
	current_image_ = cv_ptr->image;
	return current_image_;
}


void PinholeModel::rectify(cv::Mat& img, cv::Mat K_undist = cv::Mat())
{
	cv::Mat undist_img;
    cv::undistort(img, undist_img, _K_cv, _distortion, K_undist);
}

void PinholeModel::crop(cv::Mat& img, int x, int y, int patch_width, int patch_height)
{   
    cv::Mat temp = img(cv::Rect(x, y, patch_width, patch_height)).clone(); img = temp;
}




class StereoCameras
{
private:
	// Cameras
	PinholeModel _camera_left;
	PinholeModel _camera_right;

	// Crop part of image not fitting with gridsize
	int _patch_width, _patch_height;

	// Tranformation between cameras - Will be used for triangulation
	cv::Mat _P_cl, _P_cr;	

    Eigen::Affine3d _T_clcr;
    Eigen::Affine3d _T_crcl;

public:
	StereoCameras(ros::NodeHandle nh, int grid_size) 
	: _camera_left(nh, "camera_left"), _camera_right(nh, "camera_right") 
	{
		_patch_width = _camera_left.getWidth() - (_camera_left.getWidth() % grid_size);
		_patch_height = _camera_left.getHeight() - (_camera_left.getHeight() % grid_size);


		double r11, r12, r13, r21, r22, r23, r31, r32, r33;
		nh.getParam("/stereo/rotation/r11", r11);
		nh.getParam("/stereo/rotation/r12", r12);
		nh.getParam("/stereo/rotation/r13", r13);
		nh.getParam("/stereo/rotation/r21", r21);
		nh.getParam("/stereo/rotation/r22", r22);
		nh.getParam("/stereo/rotation/r23", r23);
		nh.getParam("/stereo/rotation/r31", r31);
		nh.getParam("/stereo/rotation/r32", r32);
		nh.getParam("/stereo/rotation/r33", r33);

		double x, y, z;
		nh.getParam("/stereo/translation/t1", x);
		nh.getParam("/stereo/translation/t2", y);
		nh.getParam("/stereo/translation/t3", z);


		_P_cl = cv::Mat::eye(cv::Size(4,3), CV_64FC1);
		_P_cl = _camera_left.K_cv() * _P_cl;

		_P_cr = cv::Mat::eye(cv::Size(4,3), CV_64FC1);
		_P_cr.at<double>(0,3) = x;		
		// _P_cr.at<double>(1,3) = y;		
		// _P_cr.at<double>(2,3) = z;		
		_P_cr = _camera_right.K_cv() * _P_cr;


		_T_clcr.linear() << r11, r12, r13,
							r21, r22, r23,
							r31, r32, r33;
		_T_clcr.translation() << x, 
								 y, 
								 z;

		_T_crcl.linear() << r11, r21, r31,
							r12, r22, r32,
							r13, r23, r33;
		_T_crcl.translation() << -x, 
								 -y, 
								 -z;

	};
	~StereoCameras() {};

	PinholeModel left()  {return _camera_left;};
	PinholeModel right() {return _camera_right;};

	cv::Mat leftProjMat() {return _P_cl;};
	cv::Mat rightProjMat() {return _P_cr;};
	Eigen::Affine3d getStereoTransformation() {return _T_clcr;};
	Eigen::Affine3d getInverseStereoTransformation() {return _T_crcl;};
	std::pair<cv::Mat, cv::Mat> getStereoImagePair() {return std::make_pair(_camera_left.getCurrentImage(), _camera_right.getCurrentImage());};
	std::pair<cv::Mat, cv::Mat> getStereoImagePairPrevious() {return std::make_pair(_camera_left.getPreviousImage(), _camera_right.getPreviousImage());};

	std::pair<cv::Mat, cv::Mat> storeImagePairGray(const sensor_msgs::ImageConstPtr &cam_left, const sensor_msgs::ImageConstPtr &cam_right);
	void prepareImages(cv::Mat& img_left, cv::Mat& img_right);
	cv::Mat createProjectionMatrix(Eigen::Affine3d T,
                                   Eigen::Matrix3d K);

};


std::pair<cv::Mat, cv::Mat> StereoCameras::storeImagePairGray(const sensor_msgs::ImageConstPtr &img_left_msg, 
															  const sensor_msgs::ImageConstPtr &img_right_msg)
{
	cv::Mat img_left  = _camera_left.storeGray(img_left_msg);
	cv::Mat img_right = _camera_right.storeGray(img_right_msg);
	return std::make_pair(img_left, img_right);
}


void StereoCameras::prepareImages(cv::Mat& img_left, cv::Mat& img_right)
{
	_camera_left.rectify(img_left);
	_camera_right.rectify(img_right);

	_camera_left.crop(img_left, 0, 0, _patch_width, _patch_height);
	_camera_right.crop(img_right, 0, 0, _patch_width, _patch_height);
}


cv::Mat StereoCameras::createProjectionMatrix(Eigen::Affine3d T,
											  Eigen::Matrix3d K)
{
  cv::Mat R, t, Rt, K_cv;
  Eigen::Matrix3d R_mat = T.linear();
  Eigen::Vector3d t_mat = T.translation();
  cv::eigen2cv(R_mat, R);
  cv::eigen2cv(t_mat, t);
  cv::eigen2cv(K, K_cv);
  
  cv::hconcat(R, t, Rt);
  return K_cv*Rt;
}