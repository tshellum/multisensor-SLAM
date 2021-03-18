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
#include "support.h"



class PosePredictionFeatures
{
private:

public:
    PosePredictionFeatures()
    {};

    ~PosePredictionFeatures() {};

    void removeRANSACoutliers(cv::Mat inliers, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2);

    Eigen::Affine3d estimatePoseFromFeatures(std::vector<cv::Point2f>& points_prev, std::vector<cv::Point2f>& points_cur, cv::Mat K);
    
    Eigen::Affine3d cv2eigen(cv::Mat R, cv::Mat t);
};


Eigen::Affine3d PosePredictionFeatures::cv2eigen(cv::Mat R, cv::Mat t)
{
    Eigen::Matrix3d R_eig;
    Eigen::Vector3d t_eig;
    cv::cv2eigen(R, R_eig);
    cv::cv2eigen(t, t_eig);

    Eigen::Affine3d T;
    T.linear() = R_eig;
    T.translation() = t_eig;

    return T;
}



void PosePredictionFeatures::removeRANSACoutliers(cv::Mat inliers, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
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


Eigen::Affine3d PosePredictionFeatures::estimatePoseFromFeatures(std::vector<cv::Point2f>& points_prev, std::vector<cv::Point2f>& points_cur, cv::Mat K)
{
    Eigen::Affine3d T_b1b2 = Eigen::Affine3d::Identity();

    if (points_cur.size() >= 5) // 5 for Essential, 8 for fundamental
    {
        cv::Mat E, F, inliers; 
        cv::Mat R_b1b2, t_b1b2;

        E = cv::findEssentialMat(points_prev, points_cur, K, cv::RANSAC, 0.999, 1.0, inliers); 
        removeRANSACoutliers(inliers, points_prev, points_cur);	// Track + match
        cv::recoverPose(E, points_prev, points_cur, K, R_b1b2, t_b1b2); // z = viewer direction, x and y follows camera frame
       
        return cv2eigen(R_b1b2, t_b1b2);
    }

    return T_b1b2;
}
    