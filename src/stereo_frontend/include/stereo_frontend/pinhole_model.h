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

    int _width, _height;

    // Projection parameters
    double _fx, _fy, _cx, _cy;
    
    // Distortion parameters
    double _k1, _k2, _p1, _p2, _k3;
    cv::Mat _distortion;
    
    // Calibration matrix
    cv::Mat _K_cv;
   	Eigen::Matrix3d _K_eig; 	
 
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

    void undistort(cv::Mat& img, cv::Mat K_undist);
    void crop(cv::Mat& img, int x1, int x2, int y1, int y2);
};


void PinholeModel::undistort(cv::Mat& img, cv::Mat K_undist = cv::Mat())
{
	cv::Mat undist_img;
    cv::undistort(img, undist_img, _K_cv, _distortion, K_undist);
}

void PinholeModel::crop(cv::Mat& img, int x, int y, int patch_width, int patch_height)
{   
    cv::Mat temp = img(cv::Rect(x, y, patch_width, patch_height)).clone(); img = temp;
}