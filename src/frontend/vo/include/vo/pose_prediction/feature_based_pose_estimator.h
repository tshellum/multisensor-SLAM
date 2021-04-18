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

/*** OpenCV packages ***/
#include "opencv2/core/eigen.hpp"

/*** Classes ***/
#include "support.h"



class PosePredictionFeatures
{
private:
    Eigen::Matrix4d T_cb_;

public:
    PosePredictionFeatures()
    : T_cb_(Eigen::Matrix4d::Identity())
    {
        // z in headed direction. The motion will be described in the negative direction of the camera depth
        T_cb_ << 1, 0,  0, 0,
                 0, 1,  0, 0,
                 0, 0, -1, 0,
                 0, 0,  0, 1;
    };

    ~PosePredictionFeatures() {};

    void removeRANSACoutliers(cv::Mat inliers, 
                              std::vector<cv::Point2f>& points1, 
                              std::vector<cv::Point2f>& points2,
                              std::vector<cv::KeyPoint>& kpts_prev, 
                              std::vector<cv::KeyPoint>& kpts_cur);
                            
    void removeRANSACoutliers(cv::Mat inliers, 
                              std::vector<cv::Point2f>& points1, 
                              std::vector<cv::Point2f>& points2,
                              std::vector<cv::KeyPoint>& kpts_prev, 
                              std::vector<cv::KeyPoint>& kpts_cur,
                              std::vector<cv::Point3f>& landmarks);
    
    void removeRANSACoutliers(cv::Mat inliers, 
                              std::vector<cv::Point2f>& points1, 
                              std::vector<cv::Point2f>& points2, 
                              std::vector<cv::KeyPoint>& kpts1, 
                              std::vector<cv::KeyPoint>& kpts2,
                              std::vector<cv::Point3f>& landmarks,
                              std::vector<cv::DMatch>& matches);

    Eigen::Affine3d estimatePoseFromFeatures(std::vector<cv::KeyPoint>& kpts_prev, 
                                             std::vector<cv::KeyPoint>& kpts_cur, 
                                             cv::Mat K);

    Eigen::Affine3d estimatePoseFromFeatures(std::vector<cv::KeyPoint>& kpts_prev, 
                                             std::vector<cv::KeyPoint>& kpts_cur,
                                             std::vector<cv::Point3f>& landmarks, 
                                             cv::Mat K);

    Eigen::Affine3d estimatePoseFromFeatures(std::vector<cv::KeyPoint>& kpts_prev, 
                                             std::vector<cv::KeyPoint>& kpts_cur, 
                                             std::vector<cv::Point3f>& landmarks,
                                             std::vector<cv::DMatch>& matches, 
                                             cv::Mat K);

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



void PosePredictionFeatures::removeRANSACoutliers(cv::Mat inliers, 
                                                  std::vector<cv::Point2f>& points1, 
                                                  std::vector<cv::Point2f>& points2, 
                                                  std::vector<cv::KeyPoint>& kpts1, 
                                                  std::vector<cv::KeyPoint>& kpts2)
{        
    std::vector<cv::Point2f> inlier_match_points1, inlier_match_points2;
    std::vector<cv::KeyPoint> inlier_kpts1, inlier_kpts2;

    for(int i = 0; i < inliers.rows;i++) 
    {
        if(inliers.at<unsigned char>(i))
        {
            inlier_match_points1.push_back(points1[i]);
            inlier_match_points2.push_back(points2[i]);
            inlier_kpts1.push_back(kpts1[i]);
            inlier_kpts2.push_back(kpts2[i]);
            
        }
    }
    points1 = inlier_match_points1;
    points2 = inlier_match_points2;
    kpts1 = inlier_kpts1;
    kpts2 = inlier_kpts2;
}



void PosePredictionFeatures::removeRANSACoutliers(cv::Mat inliers, 
                                                  std::vector<cv::Point2f>& points1, 
                                                  std::vector<cv::Point2f>& points2, 
                                                  std::vector<cv::KeyPoint>& kpts1, 
                                                  std::vector<cv::KeyPoint>& kpts2,
                                                  std::vector<cv::Point3f>& landmarks)
{        
    std::vector<cv::Point2f> inlier_match_points1, inlier_match_points2;
    std::vector<cv::KeyPoint> inlier_kpts1, inlier_kpts2;
    std::vector<cv::Point3f> inlier_landmarks;

    for(int i = 0; i < inliers.rows;i++) 
    {
        if(inliers.at<unsigned char>(i))
        {
            inlier_match_points1.push_back(points1[i]);
            inlier_match_points2.push_back(points2[i]);
            inlier_kpts1.push_back(kpts1[i]);
            inlier_kpts2.push_back(kpts2[i]);
            inlier_landmarks.push_back(landmarks[i]);
        }
    }
    points1 = inlier_match_points1;
    points2 = inlier_match_points2;
    kpts1 = inlier_kpts1;
    kpts2 = inlier_kpts2;
    landmarks = inlier_landmarks;
}



void PosePredictionFeatures::removeRANSACoutliers(cv::Mat inliers, 
                                                  std::vector<cv::Point2f>& points1, 
                                                  std::vector<cv::Point2f>& points2, 
                                                  std::vector<cv::KeyPoint>& kpts1, 
                                                  std::vector<cv::KeyPoint>& kpts2,
                                                  std::vector<cv::Point3f>& landmarks,
                                                  std::vector<cv::DMatch>& matches)
{        
    std::vector<cv::Point2f> inlier_match_points1, inlier_match_points2;
    std::vector<cv::KeyPoint> inlier_kpts1, inlier_kpts2;
    std::vector<cv::Point3f> inlier_landmarks;
    std::vector<cv::DMatch> inlier_matches;

    for(int i = 0; i < inliers.rows;i++) 
    {
        if(inliers.at<unsigned char>(i))
        {
            inlier_match_points1.push_back(points1[i]);
            inlier_match_points2.push_back(points2[i]);
            inlier_kpts1.push_back(kpts1[i]);
            inlier_kpts2.push_back(kpts2[i]);
            inlier_landmarks.push_back(landmarks[i]);
            inlier_matches.push_back(matches[i]);
        }
    }
    points1 = inlier_match_points1;
    points2 = inlier_match_points2;
    kpts1 = inlier_kpts1;
    kpts2 = inlier_kpts2;
    landmarks = inlier_landmarks;
    matches = inlier_matches;
}


Eigen::Affine3d PosePredictionFeatures::estimatePoseFromFeatures(std::vector<cv::KeyPoint>& kpts_prev, 
                                                                 std::vector<cv::KeyPoint>& kpts_cur, 
                                                                 cv::Mat K)
{
    std::vector<cv::Point2f> pts_prev, pts_cur;
    cv::KeyPoint::convert(kpts_prev, pts_prev);
    cv::KeyPoint::convert(kpts_cur, pts_cur);

    Eigen::Affine3d T_b1b2 = Eigen::Affine3d::Identity();

    if ((kpts_prev.size() >= 5) || (kpts_cur.size() >= 5)) // 5 for Essential, 8 for fundamental
    {
        cv::Mat E, F, inliers; 
        cv::Mat R_b1b2, t_b1b2;

        E = cv::findEssentialMat(pts_prev, pts_cur, K, cv::RANSAC, 0.999, 1.0, inliers); 
        removeRANSACoutliers(inliers, pts_prev, pts_cur, kpts_prev, kpts_cur);	// Track + match
        cv::recoverPose(E, pts_prev, pts_cur, K, R_b1b2, t_b1b2); // z = viewer direction, x and y follows camera frame

        T_b1b2 = cv2eigen(R_b1b2, t_b1b2);
        T_b1b2(2,3) *= -1;

        return T_b1b2; 
        // return Eigen::Affine3d{T_cb_ * T_b1b2.matrix() * T_cb_}; // In reality T_c * T * T_c.transpose()
    }

    return T_b1b2;
}
    

Eigen::Affine3d PosePredictionFeatures::estimatePoseFromFeatures(std::vector<cv::KeyPoint>& kpts_prev, 
                                                                 std::vector<cv::KeyPoint>& kpts_cur, 
                                                                 std::vector<cv::Point3f>& landmarks, 
                                                                 cv::Mat K)
{
    std::vector<cv::Point2f> pts_prev, pts_cur;
    cv::KeyPoint::convert(kpts_prev, pts_prev);
    cv::KeyPoint::convert(kpts_cur, pts_cur);

    Eigen::Affine3d T_b1b2 = Eigen::Affine3d::Identity();

    if ((kpts_prev.size() >= 5) || (kpts_cur.size() >= 5)) // 5 for Essential, 8 for fundamental
    {
        cv::Mat E, F, inliers; 
        cv::Mat R_b1b2, t_b1b2;

        E = cv::findEssentialMat(pts_prev, pts_cur, K, cv::RANSAC, 0.999, 1.0, inliers); 
        removeRANSACoutliers(inliers, pts_prev, pts_cur, kpts_prev, kpts_cur, landmarks);	// Track + match
        cv::recoverPose(E, pts_prev, pts_cur, K, R_b1b2, t_b1b2); // z = viewer direction, x and y follows camera frame

        T_b1b2 = cv2eigen(R_b1b2, t_b1b2);
        T_b1b2(2,3) *= -1;

        return T_b1b2; 
    }

    return T_b1b2;
}
    


Eigen::Affine3d PosePredictionFeatures::estimatePoseFromFeatures(std::vector<cv::KeyPoint>& kpts_prev, 
                                                                 std::vector<cv::KeyPoint>& kpts_cur, 
                                                                 std::vector<cv::Point3f>& landmarks,
                                                                 std::vector<cv::DMatch>& matches, 
                                                                 cv::Mat K)
{
    std::vector<cv::Point2f> pts_prev, pts_cur;
    cv::KeyPoint::convert(kpts_prev, pts_prev);
    cv::KeyPoint::convert(kpts_cur, pts_cur);

    Eigen::Affine3d T_b1b2 = Eigen::Affine3d::Identity();

    if ((kpts_prev.size() >= 5) || (kpts_cur.size() >= 5)) // 5 for Essential, 8 for fundamental
    {
        cv::Mat E, F, inliers; 
        cv::Mat R_b1b2, t_b1b2;

        E = cv::findEssentialMat(pts_prev, pts_cur, K, cv::RANSAC, 0.999, 1.0, inliers); 
        removeRANSACoutliers(inliers, pts_prev, pts_cur, kpts_prev, kpts_cur, landmarks, matches);	// Track + match
        cv::recoverPose(E, pts_prev, pts_cur, K, R_b1b2, t_b1b2); // z = viewer direction, x and y follows camera frame

        T_b1b2 = cv2eigen(R_b1b2, t_b1b2);
        T_b1b2(2,3) *= -1;

        return T_b1b2; 
    }

    return T_b1b2;
}
    