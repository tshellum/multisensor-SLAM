#pragma once

/*** ROS packages ***/
#include <ros/ros.h>
#include <ros/package.h>

/*** C++ packages ***/
#include <fstream>
#include <sys/stat.h> 

/*** Eigen packages ***/
#include <Eigen/Geometry> 


class Pose
{
private:
    // Global pose
    cv::Mat _t_wb, _R_wb, _euler_wb;
    Eigen::Quaterniond _q_wb;
    double _current_timestamp;

    // Relative pose
    cv::Mat _R_b1b2, _t_b1b2;
    double _scale;    
    
    // File
    std::string _result_path;
    std::string _filename;

public:
    Pose(std::string filename) 
    : _result_path(ros::package::getPath("stereo_frontend") + "/../../results/"),
        _filename(filename)
    {
        // Create directory
        mkdir(_result_path.c_str(), 0777);

        // Clear files
        std::ofstream vo_results;
        vo_results.open(_result_path + _filename, std::ofstream::out | std::ofstream::trunc);
        vo_results.close();

        _t_wb = cv::Mat::zeros(3,1, CV_64F);
		_R_wb = cv::Mat::eye(3,3, CV_64F);
    };

    ~Pose() {};

    void setWorldFrame(cv::Mat t_wb0, cv::Mat R_wb0);

    void updatePose();

    void pose2file();

    // TODO: Formulate remaining functions after main system structure is in place
};


void Pose::updatePose()
{
    _t_wb = _t_wb + _R_wb * _t_b1b2 * _scale; 
	_R_wb = _R_wb * _R_b1b2;
}


void Pose::setWorldFrame(cv::Mat t_wb0, cv::Mat R_wb0)
{
    _t_wb = t_wb0;
    _R_wb = R_wb0;
}


void Pose::pose2file()
{
    // Ground truth
    std::ofstream results;
    results.open (_result_path + _filename, std::ios_base::app);
    results << _current_timestamp  << " "   // timestamp for current pose
            << _t_wb.at<double>(0) << " "	// x translation 
            << _t_wb.at<double>(1) << " "	// y translation
            << _t_wb.at<double>(2) << " "	// z translation 
            << _q_wb.x()		 	   << " "	// x quaternion
            << _q_wb.y()		 	   << " "	// y quaternion
            << _q_wb.z()		 	   << " "	// y quaternion
            << _q_wb.w() 		   << "\n";	// w quaternion

    results.close();
}