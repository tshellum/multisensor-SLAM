#pragma once
#include <ros/ros.h>

#include <string>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/calib3d.hpp>


class PinholeModel
{
private:
    ros::NodeHandle nh_;
    
    std::string name_;

    int width_, height_;

    // Projection parameters
    double fx_, fy_, cx_, cy_;
    
    // Distortion parameters
    double k1_, k2_, p1_, p2_;


    cv::Mat K_cv;
    // TODO: parametes as camera matrix - eigen
   	// Eigen::Matrix3d K_eig; 	
 
public:
    // PinholeModel()
    // {
    //     ROS_INFO("pinholemodel constructed");
    //     print_s("pinhole mod");
    // };

    // void print_s(std::string name){ ROS_INFO_STREAM(name); };

    // PinholeModel(const std::string camera_name) : name_(camera_name)
    // {
    //     ROS_INFO_STREAM("pinholemodel constructed: " + name_);
    // }

    

    PinholeModel() : name_("camera_left")
    {
        nh_.getParam("/"+name_+"/image_width", width_);
        nh_.getParam("/"+name_+"/image_height", height_);
        nh_.getParam("/"+name_+"/projection_parameters/fx", fx_);
        nh_.getParam("/"+name_+"/projection_parameters/fy", fy_);
        nh_.getParam("/"+name_+"/projection_parameters/cx", cx_);
        nh_.getParam("/"+name_+"/projection_parameters/cy", cy_);
        nh_.getParam("/"+name_+"/distortion_parameters/k1", k1_);
        nh_.getParam("/"+name_+"/distortion_parameters/k2", k2_);
        nh_.getParam("/"+name_+"/distortion_parameters/p1", p1_);
        nh_.getParam("/"+name_+"/distortion_parameters/p2", p2_);

        K_cv = (cv::Mat_<double>(3,3) << 
                fx_,   0, cx_,
                  0, fy_, cy_,
                  0,   0,   1);
        
        // cv::cv2eigen(K_cv, K_eig);
    };

    ~PinholeModel() {};

    cv::Mat get_K() {return K_cv;};
};



