#pragma once

/*** ROS packages ***/
#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*** C++ packages ***/
#include <fstream>
#include <sys/stat.h> 

/*** Eigen packages ***/
#include <Eigen/Geometry> 
#include <Eigen/Dense>

/*** Classes ***/
#include "stereo_frontend/support.h"


// TODO: #include "affine.hpp"
// cv::Affine3d cv_T_W_V(cv_R_W_V, cv_t_W_V);

class Pose
{
private:
    // Global pose
    cv::Mat _R_wb0, _t_wb0, _T_wb0; // World center
    cv::Mat _R_wb, _t_wb, _T_wb;
    bool _is_set;

    // Relative pose
    cv::Mat _R_b1b2, _t_b1b2, _T_b1b2;
    double _scale;    

    // To correct global frame error
    cv::Mat _R_gnss;

    // Static transformation between body and camera 
    cv::Mat _T_cam, _T_bc, _T_cb; 

    // Time
    double _start_time;
    double _current_timestamp;

    // File
    std::string _result_path;
    std::string _filename;

public:
    Pose(ros::NodeHandle nh, std::string filename) 
    : _result_path(ros::package::getPath("stereo_frontend") + "/../../results/"),
      _filename(filename), _scale(0.0), _start_time(0.0), _current_timestamp(0.0),
      _is_set(false)
    {
        // Create directory
        mkdir(_result_path.c_str(), 0777);

        // Clear files
        std::ofstream vo_results;
        vo_results.open(_result_path + _filename, std::ofstream::out | std::ofstream::trunc);
        vo_results.close();

        _t_b1b2 = cv::Mat::zeros(3,1, CV_64F);
        _R_b1b2 = cv::Mat::eye(3,3, CV_64F);
        _T_b1b2 = cv::Mat::eye(4,4, CV_64F);

        _t_wb = cv::Mat::zeros(3,1, CV_64F);
        _R_wb = cv::Mat::eye(3,3, CV_64F);
        _T_wb = cv::Mat::eye(4,4, CV_64F);

        _T_cam = (cv::Mat_<double>(4,4) << 1, 0,  0, 0,
                                           0, 1,  0, 0,
                                           0, 0, -1, 0,
                                           0, 0,  0, 1);

        _T_bc = (cv::Mat_<double>(4,4) << 1, 0, 0, 0,
                                          0, 0, 1, 0,
                                          0, 1, 0, 0,
                                          0, 0, 0, 1);
        // _T_bc << 1,  0,  0, 0,
        //          0,  0, -1, 0,
        //          0, -1,  0, 0,
        //          0,  0,  0, 1;

        // _T_bc << 0, 0, 1, 0,
        //          1, 0, 0, 0,
        //          0, 1, 0, 0,
        //          0, 0, 0, 1;

        _T_cb = _T_bc.t();

        std::string frame[3] = {"x", "y", "z"};
        nh.getParam("/GNSS_frame/surge", frame[0]);
        nh.getParam("/GNSS_frame/sway", frame[1]);
        nh.getParam("/GNSS_frame/yaw", frame[2]);


        _R_gnss = cv::Mat::zeros(3,3, CV_64F);
        for(int i = 0; i < 3; i++)
        {
            if (frame[i] == "x") 
                _R_gnss.at<double>(0,i) = 1;
            else if (frame[i] == "y")
                _R_gnss.at<double>(1,i) = 1;
            else if (frame[i] == "z")
                _R_gnss.at<double>(2,i) = 1;
            else
                _R_gnss.at<double>(i,i) = 1;
        }

        // _R_gnss << 0,  0, -1,
        //            1,  0,  0,
        //            0, -1,  0;
    };

    ~Pose() {};

    cv::Mat getRelativeRotation()       {return _R_b1b2;};
    cv::Mat getRelativeTranslation()    {return _t_b1b2;};
    cv::Mat getRelativeTransformation() {return _T_b1b2;};
    cv::Mat getWorldRotation()          {return _R_wb;};
    cv::Mat getWorldTranslation()       {return _t_wb;};
    cv::Mat getWorldTransformation()    {return _T_wb;};
    double getTimestamp()               {return _current_timestamp;};
    double getScale()                   {return _scale;};
    bool isSet()                        {return _is_set;};

    void readScale(double scale) {_scale = scale;};    
    void readTimestamp(double time);
    void readRotation(double qx, double qy, double qz,double qw);
    void readTranslation(double x, double y, double z);

    cv::Mat constructTransformation(cv::Mat R, cv::Mat t);
    cv::Mat getRotation(cv::Mat T) {return T(cv::Rect(0, 0, 3, 3));};
    cv::Mat getTranslation(cv::Mat T) {return T(cv::Rect(3, 0, 1, 3));};

    void setWorldCenter(cv::Mat R_wb0, cv::Mat t_wb0);
    void correctGNSSFrame();
    void estimateScaleFromGNSS(cv::Mat t, cv::Mat t_kf);
    
    bool isValidRotation(double thresh);
    bool isValidTranslation(double thresh);
    void removeRANSACoutliers(cv::Mat inliers, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2);
    bool initialPoseEstimate(std::vector<cv::Point2f>& points_prev, std::vector<cv::Point2f>& points_cur, cv::Mat K);
    bool PnP(std::vector<cv::Point2f> img_pts, std::vector<cv::Point3f> world_pts, cv::Mat K, cv::Mat distortion);
    void updatePose();

    void toFile();
    geometry_msgs::PoseStamped toPoseStamped(std_msgs::Header header, cv::Mat T);

    void operator = (Pose const &obj);
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
    Eigen::Quaterniond q_wb;
    q_wb.x() = qx;
    q_wb.y() = qy;
    q_wb.z() = qz;
    q_wb.w() = qw;

    Eigen::Matrix3d R_wb;
    R_wb = q_wb.normalized().toRotationMatrix();
    cv::eigen2cv(R_wb, _R_wb);
}


void Pose::readTranslation(double x, double y, double z)
{
    _t_wb.at<double>(0) = x;
    _t_wb.at<double>(1) = y;
    _t_wb.at<double>(2) = z;
}


cv::Mat Pose::constructTransformation(cv::Mat R, cv::Mat t) 
{
    cv::Mat T = cv::Mat::eye(4,4, CV_64F); 
    R.copyTo(T(cv::Rect(0, 0, 3, 3))); 
    t.copyTo(T(cv::Rect(3, 0, 1, 3)));

    return T;
}


// TODO: Skrive om til transformasjoner
void Pose::updatePose()
{
    _t_wb = (_t_wb + _R_wb * _t_b1b2); 
	_R_wb = _R_wb * _R_b1b2;
    
    _T_wb = constructTransformation(_R_wb, _t_wb);
}


void Pose::setWorldCenter(cv::Mat R_wb0, cv::Mat t_wb0)
{
    _R_wb = _R_wb0 * _R_wb;
    _t_wb = _R_wb0 * _t_wb + _t_wb0;
    _T_wb = constructTransformation(_R_wb, _t_wb);

    _is_set = true;
}


void Pose::correctGNSSFrame()
{ 
    _R_wb *= _R_gnss;
    _t_wb = _R_gnss * _t_wb;
}


void Pose::estimateScaleFromGNSS(cv::Mat t, cv::Mat t_kf)
{
    double scale = sqrt( pow(t.at<double>(0) - t_kf.at<double>(0), 2)    // x²
                       + pow(t.at<double>(1) - t_kf.at<double>(1), 2)    // y²
                       + pow(t.at<double>(2) - t_kf.at<double>(2), 2) ); // z²

    if (scale != 0) // if not same GNSS measurement
        _scale = scale;
}


bool Pose::isValidRotation(double thresh = M_PI/3)
{
    cv::Mat euler = cv::Mat::zeros(3,1, CV_64F);
    cv::Rodrigues(_R_b1b2, euler);
    
    double phi   = euler.at<double>(0);
    double theta = euler.at<double>(1);
    double psi   = euler.at<double>(2);
    return (-thresh <= phi && phi <= thresh) && (-thresh <= theta && theta <= thresh) && (-thresh <= psi && psi <= thresh);
}


bool Pose::isValidTranslation(double thresh = 0.8)
{
    double x = _t_b1b2.at<double>(0);
    double y = _t_b1b2.at<double>(1);
    double z = _t_b1b2.at<double>(2);

    return (-thresh <= y && y <= thresh) && (0 <= x); 
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
        removeRANSACoutliers(inliers, points_prev, points_cur);	// Track + match
        cv::recoverPose(E, points_prev, points_cur, K, _R_b1b2, _t_b1b2); // z = viewer direction, x and y follows camera frame
        _t_b1b2 *= _scale;
       
        _T_b1b2 = constructTransformation(_R_b1b2, _t_b1b2);
        // _T_b1b2 = _T_cam * _T_b1b2 * _T_cam;
        _R_b1b2 = getRotation(_T_b1b2);
        _t_b1b2 = getTranslation(_T_b1b2);

        if ((!isValidRotation()) || (!isValidTranslation() || _scale > 0.1))
            return false;

        return true;
    }
    else
    {
        ROS_INFO("Too few features detected for pose estimation...");
        return false;
    }
    
}

bool Pose::PnP(std::vector<cv::Point2f> img_pts, std::vector<cv::Point3f> world_pts, cv::Mat K, cv::Mat distortion)
{
    cv::Mat R,t;
    // cv::solvePnP(world_pts, img_pts, K, distortion, R, t, false, cv::SOLVEPNP_EPNP);
    
    return false;
}



void Pose::toFile()
{
    cv::Mat T = _T_bc * _T_wb * _T_cb;
    cv::Mat R = T(cv::Rect(0, 0, 3, 3));
    cv::Mat t = T(cv::Rect(3, 0, 1, 3));

    Eigen::Matrix3d R_wb;
    cv::cv2eigen(R, R_wb);
    Eigen::Quaterniond q_wb;
    q_wb = R_wb;

    // Ground truth
    std::ofstream results;
    results.open (_result_path + _filename, std::ios_base::app);
    results << _current_timestamp  << " "   // timestamp for current pose
            << _t_wb.at<double>(0)      << " "	// x translation 
            << _t_wb.at<double>(0)      << " "	// y translation
            << _t_wb.at<double>(0)      << " "	// z translation 
            << q_wb.x()	       << " "	// x quaternion
            << q_wb.y()	       << " "	// y quaternion
            << q_wb.z()	       << " "	// y quaternion
            << q_wb.w() 	       << "\n";	// w quaternion

    results.close();
}


geometry_msgs::PoseStamped Pose::toPoseStamped(std_msgs::Header header, cv::Mat T)
{
    T = _T_bc * T * _T_cb;
    cv::Mat R = T(cv::Rect(0, 0, 3, 3));
    cv::Mat t = T(cv::Rect(3, 0, 1, 3));

    tf2::Matrix3x3 tf2_rot(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                           R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                           R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
    // Create a transform and convert to a Pose
    tf2::Transform tf2_transform(tf2_rot, tf2::Vector3(t.at<double>(0), 
                                                       t.at<double>(1), 
                                                       t.at<double>(2))
    );
    
    geometry_msgs::Pose pose_msg;
    tf2::toMsg(tf2_transform, pose_msg);

    geometry_msgs::PoseStamped stamped_pose_msg; 
    stamped_pose_msg.pose = pose_msg; 
    stamped_pose_msg.header.stamp = header.stamp;
    stamped_pose_msg.header.frame_id = "relative_pose";
    return stamped_pose_msg; 
}


void Pose::operator = (Pose const &obj) 
{ 
    // Global pose
    _R_wb0 = obj._R_wb0.clone();
    _t_wb0 = obj._t_wb0.clone();
    _T_wb0 = obj._T_wb0.clone();

    _R_wb = obj._R_wb.clone();
    _t_wb = obj._t_wb.clone();
    _T_wb = obj._T_wb.clone();
    _is_set = obj._is_set;

    // Relative pose
    _R_b1b2 = obj._R_b1b2.clone();
    _t_b1b2 = obj._t_b1b2.clone();
    _T_b1b2 = obj._T_b1b2.clone();
    _scale = obj._scale;

    // To correct global frame error
    _R_gnss = obj._R_gnss.clone();

    // Static transformation between body and camera 
    _T_bc = obj._T_bc.clone();
    _T_cb = obj._T_cb.clone();

    // Time
    _start_time = obj._start_time;
    _current_timestamp = obj._current_timestamp;
} 