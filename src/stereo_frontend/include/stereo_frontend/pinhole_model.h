#pragma once

#include <string>
#include <ros/ros.h>
#include <Eigen/Dense> 
#include "opencv2/core/eigen.hpp"


class PinholeModel
{
private:    
    std::string name_;

    int width_, height_;

    // Projection parameters
    double fx_, fy_, cx_, cy_;
    
    // Distortion parameters
    double k1_, k2_, p1_, p2_, k3_;
    cv::Mat distortion_;
    
    // Calibration matrix
    cv::Mat K_cv_;
   	Eigen::Matrix3d K_eig_; 	
 
public:
    PinholeModel(const std::string name, ros::NodeHandle nh) : name_(name), k3_(0.0)
    {
        nh.getParam("/"+name_+"/image_width", width_);
        nh.getParam("/"+name_+"/image_height", height_);
        nh.getParam("/"+name_+"/projection_parameters/fx", fx_);
        nh.getParam("/"+name_+"/projection_parameters/fy", fy_);
        nh.getParam("/"+name_+"/projection_parameters/cx", cx_);
        nh.getParam("/"+name_+"/projection_parameters/cy", cy_);
        nh.getParam("/"+name_+"/distortion_parameters/k1", k1_);
        nh.getParam("/"+name_+"/distortion_parameters/k2", k2_);
        nh.getParam("/"+name_+"/distortion_parameters/p1", p1_);
        nh.getParam("/"+name_+"/distortion_parameters/p2", p2_);

        K_cv_ = (cv::Mat_<double>(3,3) << 
                fx_,   0, cx_,
                  0, fy_, cy_,
                  0,   0,   1);
        
        cv::cv2eigen(K_cv_, K_eig_);

        distortion_ = (cv::Mat_<double>(5,1) << k1_, k2_, p1_, p2_, k3_);
    };

    ~PinholeModel() {};


    cv::Mat K_cv() {return K_cv_;};
    Eigen::Matrix3d K_eig() {return K_eig_;};
    cv::Mat distortion() {return distortion_;};
};



