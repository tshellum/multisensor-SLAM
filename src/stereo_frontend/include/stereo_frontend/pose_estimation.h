#pragma once

/*** ROS packages ***/
#include <ros/ros.h>
#include <ros/package.h>

/*** C++ packages ***/
#include <fstream>
#include <sys/stat.h> 

/*** Eigen packages ***/
#include <Eigen/Geometry> 
#include <Eigen/Dense>

/*** Classes ***/
#include "stereo_frontend/support.h"

class Pose
{
private:
    // Global pose
    Eigen::Matrix3d    _R_wb;
    Eigen::Quaterniond _q_wb;
    Eigen::Vector3d    _t_wb;
    // Eigen::Matrix4d    _T_wb;
    
    // To correct global frame error
    Eigen::Matrix3d _R_gnss;

    // Relative pose
    Eigen::Matrix3d _R_b1b2;
    Eigen::Vector3d _t_b1b2;
    Eigen::Matrix4d _T_b1b2;
    double _scale;    

    // Static transformation
    Eigen::Matrix4d _T_bc, _T_cb;   // Transformation between body and camera 

    // Time
    double _start_time;
    double _current_timestamp;

    // File
    std::string _result_path;
    std::string _filename;

public:
    Pose(ros::NodeHandle nh, std::string filename) 
    : _result_path(ros::package::getPath("stereo_frontend") + "/../../results/"),
      _filename(filename), _scale(0.0), _start_time(0.0), _current_timestamp(0.0)
    {
        // Create directory
        mkdir(_result_path.c_str(), 0777);

        // Clear files
        std::ofstream vo_results;
        vo_results.open(_result_path + _filename, std::ofstream::out | std::ofstream::trunc);
        vo_results.close();

        _t_b1b2.setZero();
        _R_b1b2.setIdentity();
        _T_b1b2.setIdentity();
    
        _t_wb.setZero();
        _R_wb.setIdentity();
        _q_wb = _R_wb;
        
        _T_bc << 0, 0, 1, 0,
                 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 0, 1;
        _T_cb = _T_bc.transpose();

        std::string frame[3] = {"x", "y", "z"};
        nh.getParam("/GNSS_frame/surge", frame[0]);
        nh.getParam("/GNSS_frame/sway", frame[1]);
        nh.getParam("/GNSS_frame/yaw", frame[2]);

        _R_gnss = Eigen::Matrix3d::Zero();
        for(int i = 0; i < 3; i++)
        {
            if (frame[i] == "x") 
                _R_gnss(0,i) = 1;
            else if (frame[i] == "y")
                _R_gnss(1,i) = 1;
            else if (frame[i] == "z")
                _R_gnss(2,i) = 1;
            else
                _R_gnss(i,i) = 1;
        }  

    };

    ~Pose() {};

    Eigen::Matrix3d getRelativeRoation()     {return _R_b1b2;};
    Eigen::Vector3d getRelativeTranslation() {return _t_b1b2;};
    Eigen::Matrix3d getWorldRotation()       {return _R_wb;};
    Eigen::Quaterniond getWorldQuaternion()  {return _q_wb;};
    Eigen::Vector3d getWorldTranslation()    {return _t_wb;};
    double getTimestamp()                    {return _current_timestamp;};
    double getScale()                        {return _scale;};

    void readScale(double scale) {_scale = scale;};    
    void readTimestamp(double time);
    void readRotation(double qx, double qy, double qz,double qw);
    void readTranslation(double x, double y, double z);

    void setWorldFrame(Eigen::Matrix3d R_wb0, Eigen::Vector3d t_wb0);
    void transformCamera2Body(Eigen::Matrix3d R_c1c2, Eigen::Vector3d t_c1c2);
    void correctGNSSFrame();
    void estimateScaleFromGNSS(Eigen::Vector3d t, Eigen::Vector3d t_kf);
    
    bool isValidRotation(double thresh);
    void removeRANSACoutliers(cv::Mat inliers, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2);
    bool initialPoseEstimate(std::vector<cv::Point2f>& points_prev, std::vector<cv::Point2f>& points_cur, cv::Mat K);
    void updatePose();

    void toFile();

};


void Pose::readTimestamp(double time)
{
    if (_start_time == 0)
        _start_time = time;
    else
        _current_timestamp = time - _start_time;
}


void Pose::readRotation(double qx, double qy, double qz,double qw)
{
    _q_wb.x() = qx;
    _q_wb.y() = qy;
    _q_wb.z() = qz;
    _q_wb.w() = qw;

    _R_wb = _q_wb.normalized().toRotationMatrix();
}


void Pose::readTranslation(double x, double y, double z)
{
    _t_wb << x, 
             y, 
             z;
}


void Pose::updatePose()
{
    _t_wb = _t_wb + _R_wb * _t_b1b2 * _scale; 
	_R_wb = _R_wb * _R_b1b2;
    _q_wb = _R_wb;  // Update quaternion
}


void Pose::setWorldFrame(Eigen::Matrix3d R_wb0, Eigen::Vector3d t_wb0)
{
    _t_wb = t_wb0;
    _R_wb = R_wb0;
}


void Pose::transformCamera2Body(Eigen::Matrix3d R_c1c2, Eigen::Vector3d t_c1c2)
{
    Eigen::Matrix4d T_c1c2 = Eigen::Matrix4d::Identity(4,4);
    T_c1c2.block<3,3>(0,0) = R_c1c2;
    T_c1c2.block<3,1>(0,3) = t_c1c2;

    _T_b1b2 = _T_bc * T_c1c2 * _T_cb;

    _R_b1b2 = _T_b1b2.block<3,3>(0,0);
    _t_b1b2 = _T_b1b2.col(3).head(3);
}


void Pose::correctGNSSFrame()
{ 
    _R_wb *= _R_gnss;
    _t_wb = _R_gnss * _t_wb;
}


void Pose::estimateScaleFromGNSS(Eigen::Vector3d t, Eigen::Vector3d t_kf)
{
    double scale = sqrt( pow(t.coeff(0) - t_kf.coeff(0), 2) // x²
                 + pow(t.coeff(1) - t_kf.coeff(1), 2)       // y²
                 + pow(t.coeff(2) - t_kf.coeff(2), 2) );    // z²

    if (scale != 0) // if not same GNSS measurement
        _scale = scale;
}



bool Pose::isValidRotation(double thresh = M_PI/4)
{
    cv::Mat euler = cv::Mat::zeros(3,1, CV_64F);
    cv::Mat R_b1b2;
    cv::eigen2cv(_R_b1b2, R_b1b2);

    cv::Rodrigues(R_b1b2, euler);
    
    double phi = euler.at<double>(0);
    double theta = euler.at<double>(1);
    double psi = euler.at<double>(2);
    return (-thresh <= phi && phi <= thresh) && (-thresh <= theta && theta <= thresh) && (-thresh <= psi && psi <= thresh);
}




void Pose::removeRANSACoutliers(cv::Mat inliers, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
{        
    std::vector<cv::Point2f> inlier_match_points1, inlier_match_points2;

    for(int i = 0; i < inliers.rows;i++) 
    {
        if(inliers.at<unsigned char>(i))
        {
            inlier_match_points1.push_back(points1[i]);
            inlier_match_points2.push_back(points2[i]);
        }
    }
    points1 = inlier_match_points1;
    points2 = inlier_match_points2;
}


bool Pose::initialPoseEstimate(std::vector<cv::Point2f>& points_prev, std::vector<cv::Point2f>& points_cur, cv::Mat K)
{
    if (points_cur.size() >= 8) // 5 for Essential, 8 for fundamental
    {
        cv::Mat E, F, inliers; 

        E = cv::findEssentialMat(points_prev, points_cur, K, cv::RANSAC, 0.999, 1.0, inliers); 
	    F = cv::findFundamentalMat(points_prev, points_cur, cv::RANSAC, 0.999, 1.0); 		
    
        removeRANSACoutliers(inliers, points_prev, points_cur);	// Track + match

        cv::Mat R_c1c2_opencv, t_c1c2_opencv;
        cv::recoverPose(E, points_prev, points_cur, K, R_c1c2_opencv, t_c1c2_opencv); // z = viewer direction, x and y follows camera frame
        
        Eigen::Matrix3d R_c1c2;
        Eigen::Vector3d t_c1c2;
        cv::cv2eigen(R_c1c2_opencv, R_c1c2);
        cv::cv2eigen(R_c1c2_opencv, t_c1c2);
        
        transformCamera2Body(R_c1c2, t_c1c2);
        
        if (!isValidRotation())
            return false;

        return true;
    }
    else
    {
        ROS_INFO("Too few features detected for pose estimation...");
        return false;
    }
    
}


void Pose::toFile()
{
    // Ground truth
    std::ofstream results;
    results.open (_result_path + _filename, std::ios_base::app);
    results << _current_timestamp  << " "   // timestamp for current pose
            << _t_wb.coeff(0) << " "	// x translation 
            << _t_wb.coeff(1) << " "	// y translation
            << _t_wb.coeff(2) << " "	// z translation 
            << _q_wb.x()		 	   << " "	// x quaternion
            << _q_wb.y()		 	   << " "	// y quaternion
            << _q_wb.z()		 	   << " "	// y quaternion
            << _q_wb.w() 		   << "\n";	// w quaternion

    results.close();
}