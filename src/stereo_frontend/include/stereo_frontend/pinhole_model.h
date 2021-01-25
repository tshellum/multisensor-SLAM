#pragma once

#include <string>
#include <ros/ros.h>
#include <Eigen/Dense> 
#include "opencv2/core/eigen.hpp"

#include "stereo_frontend/support.h"


class PinholeModel
{
private:    
    std::string name_;

    // Projection parameters
    double _fx, _fy, _cx, _cy;
    int _width, _height;
    
    // Distortion parameters
    double _k1, _k2, _p1, _p2, _k3;
    cv::Mat _distortion;
    
    // Calibration matrix
    cv::Mat _K_cv;
   	Eigen::Matrix3d _K_eig; 	

    // Extrinsic parameters
    cv::Mat _R_clcr, _t_clcr;

public:
    PinholeModel(ros::NodeHandle nh, const std::string name) : name_(name), _k3(0.0)
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

    void rectify(cv::Mat& img, cv::Mat K_undist);
    void crop(cv::Mat& img, int x1, int x2, int y1, int y2);
};


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
	cv::Mat _R_clcr, _t_clcr;
	cv::Mat _R_crcl, _t_crcl;

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

		_R_clcr = (cv::Mat_<double>(3,3) << r11, r12, r13,
																				r21, r22, r23,
																				r31, r32, r33);

		double x, y, z;
		nh.getParam("/stereo/translation/t1", x);
		nh.getParam("/stereo/translation/t2", y);
		nh.getParam("/stereo/translation/t3", z);

		_t_clcr = (cv::Mat_<double>(3,1) << x, 
																				y, 
																				z);


		_R_crcl = _R_clcr.t();
		_t_crcl = -_t_clcr;
	};
	~StereoCameras() {};

	PinholeModel left()  {return _camera_left;};
	PinholeModel right() {return _camera_right;};

	cv::Mat getStereoRotation() {return _R_clcr;};
	cv::Mat getStereoTranslation() {return _t_clcr;};

	void prepareImages(cv::Mat& img_left, cv::Mat& img_right);
	void calculatePerspectiveMatrix(cv::Mat& P_l, cv::Mat& P_r);
};


void StereoCameras::prepareImages(cv::Mat& img_left, cv::Mat& img_right)
{
	_camera_left.rectify(img_left);
	_camera_right.rectify(img_right);

	_camera_left.crop(img_left, 0, 0, _patch_width, _patch_height);
	_camera_right.crop(img_right, 0, 0, _patch_width, _patch_height);
}


void StereoCameras::calculatePerspectiveMatrix(cv::Mat& P_l, cv::Mat& P_r)
{
	cv::Mat Rt_l = cv::Mat::eye(3, 4, CV_64FC1);
	cv::Mat Rt_r = cv::Mat::eye(3, 4, CV_64FC1);
	cv::hconcat(_R_clcr, _t_clcr, Rt_l);

	// ROS_INFO_STREAM("_camera_left.K_cv()*Rt_l: " << _camera_left.K_cv()*Rt_l);
	// ROS_INFO_STREAM("_camera_right.K_cv()*Rt_r: " << _camera_right.K_cv()*Rt_r);

	cv::Mat R, t, Q;
	cv::stereoRectify(_camera_left.K_cv(), 
										_camera_left.distortion(),
										_camera_right.K_cv(), 
										_camera_right.distortion(),
										cv::Size(_camera_left.getWidth(), _camera_left.getHeight()),
										_R_clcr,
										_t_clcr,
										R,
										t,
										P_l,
										P_r,
										Q);

	// ROS_INFO_STREAM("R: " << R); 
	// ROS_INFO_STREAM("t: " << t);

	// ROS_INFO_STREAM("P_l: " << P_l);
	// ROS_INFO_STREAM("P_r: " << P_r << "\n");
	
	P_l = _camera_left.K_cv()*Rt_l;
	P_r = _camera_right.K_cv()*Rt_r;
}
