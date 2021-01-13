#pragma once
#include <ros/ros.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/core/eigen.hpp"

#include <Eigen/Dense> 
#include "sophus/se3.hpp"

#include <vector> 
#include "math.h"
#include <cmath>



void removeRANSACoutliers(std::vector<cv::DMatch>& matches, cv::Mat inliers, 
                        std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
{        
    
    std::vector<cv::Point2f> inlier_match_points1, inlier_match_points2;
    std::vector<cv::DMatch> match_inliers;

    // int numOutliers;
    for(int i = 0; i < inliers.rows;i++) 
    {
        if(inliers.at<unsigned char>(i))
        {
            inlier_match_points1.push_back(points1[i]);
            inlier_match_points2.push_back(points2[i]);
            match_inliers.push_back(matches[i]);
        }
        // else
        // {
        //     matches.erase(matches.begin() + i - numOutliers++);

        //     // ROS_INFO_STREAM("number of outliers: " << numOutliers );
        // }

    }
    points1 = inlier_match_points1;
    points2 = inlier_match_points2;
    matches = match_inliers;
}


void removeRANSACoutliers(cv::Mat inliers, 
                        std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
{        
    
    std::vector<cv::Point2f> inlier_match_points1, inlier_match_points2;

    // int numOutliers;
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






cv::Mat estimateFundamental(std::vector<cv::DMatch> matches, 
                std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2)
{
    cv::Mat F, mask; 
    
    // https://docs.opencv.org/master/d9/d0c/group__calib3d.html
    if (matches.size() >= 8)
    {
        F = cv::findFundamentalMat(points1, points2, cv::RANSAC, 0.999, 1.0); 
    }


    return F;
}


double estimateScale(const std::vector<cv::Point2f>& image_points,
             const std::vector<cv::Point2f>& world_points,
             Eigen::Matrix3d K_)
{
    // Set a minimum required number of points,
    // here 3 times the theoretic minimum.
    // constexpr size_t min_number_points = 12;

    // Check that we have enough points.
    // if (image_points.size() < min_number_points)
    // {
    //     return {};
    // }

    // Compute the homography and extract the inliers.
    std::vector<char> inliers;
    cv::Mat H_cv = cv::findHomography(world_points, image_points, cv::RANSAC, 3, inliers);

    // std::vector<cv::Point2f> inlier_image_points;
    // std::vector<cv::Point2f> inlier_world_points;
    // for (size_t i=0; i<inliers.size(); ++i)
    // {
    //     if (inliers[i] > 0)
    //     {
    //         inlier_image_points.push_back(image_points[i]);
    //         inlier_world_points.push_back(world_points[i]);
    //     }
    // }
    
    // ROS_INFO_STREAM("HOMOGRAPHY: inlier_image_points.size(): " << inlier_image_points.size());

    // Check that we have enough inliers.
    // if (inlier_image_points.size() < min_number_points)
    // {
    //     return {};
    // }

    // ROS_INFO_STREAM("\n\nHomography: " << H_cv);
    // Convert homography to Eigen matrix.
    Eigen::Matrix3d H;
    cv::cv2eigen(H_cv, H);

    // ROS_INFO_STREAM("Homography eigen: " << H);

    // Compute the matrix M
    // and extract M_bar (the two first columns of M).
    Eigen::Matrix3d M = K_.inverse() * H;
    Eigen::MatrixXd M_bar = M.leftCols<2>();
    
    // ROS_INFO_STREAM("M: " << M);
    // ROS_INFO_STREAM("M_bar: " << M_bar);

    // Perform SVD on M_bar.
    auto svd = M_bar.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Compute R_bar (the two first columns of R)
    // from the result of the SVD.
    Eigen::Matrix<double, 3, 2> R_bar = svd.matrixU() * svd.matrixV().transpose();

    // Construct R by inserting R_bar and
    // computing the third column of R from the two first.
    // Remember to check det(R)!
    Eigen::Matrix3d R;
    R.leftCols<2>() = R_bar;
    R.col(2) = R_bar.col(0).cross(R_bar.col(1));

    if (R.determinant() < 0)
    {
        R.col(2) *= -1.0;
    }

    // Compute the scale factor lambda.
    double lambda = (R_bar.array() * M_bar.array()).sum() / (M_bar.array() * M_bar.array()).sum(); 

    // Extract the translation t.
    Eigen::Vector3d t = M.col(2) * lambda;

    // Check that this is the correct solution
    // by testing the last element of t.
    if (t.z() < 0)
    {
        // Switch to other solution.
        t = -t;
        R.topLeftCorner<3,2>() *= -1.0;
    }
    
    // ROS_INFO_STREAM("t homo: " << t);
    // ROS_INFO_STREAM("scale homo: " << lambda);


    // // Return camera pose in the world.
    // Sophus::SE3d pose_C_W(R, t);
    // return {pose_C_W.inverse(), inlier_image_points, inlier_world_points};

    return lambda;
}





geometry_msgs::PoseStamped cvPose2msgStamped(cv::Mat R, cv::Mat t)
{
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

    geometry_msgs::PoseStamped msg; 
    msg.pose = pose_msg; 
    return msg; 
}


bool isValidRotation(cv::Mat R, double thresh = M_PI/4)
{
    cv::Mat euler = cv::Mat::zeros(3,1, CV_64F);
    cv::Rodrigues(R, euler);

    // double thresh = M_PI/4;
    
    double phi = euler.at<double>(0);
    double theta = euler.at<double>(1);
    double psi = euler.at<double>(2);
    return (-thresh <= phi && phi <= thresh) && (-thresh <= theta && theta <= thresh) && (-thresh <= psi && psi <= thresh);
}

bool isValidTranslation(cv::Mat t, double thresh)
{
    double x = t.at<double>(0);
    double y = t.at<double>(1);
    double z = t.at<double>(2);

    // return (-x_thresh <= x && x <= x_thresh) && (0 >= y); // y shows negative values in positive direction...
    return (-thresh <= x && x <= thresh) && (0 <= z); // y shows negative values in positive direction...
}


bool isValidRotation_cam(cv::Mat R, double thresh = M_PI/4)
{
    cv::Mat euler = cv::Mat::zeros(3,1, CV_64F);
    cv::Rodrigues(R, euler);
    
    double phi = euler.at<double>(0);
    double theta = euler.at<double>(1);
    double psi = euler.at<double>(2);
    // return (-thresh <= theta && theta <= thresh);
    return (-thresh <= phi && phi <= thresh) && (-thresh <= theta && theta <= thresh) && (-thresh <= psi && psi <= thresh);
}


bool isValidTranslation_cam_fl(cv::Mat t, double thresh = 0.5)
{
    double x = t.at<double>(0);
    double y = t.at<double>(1);
    double z = t.at<double>(2);

    return (-thresh <= z && z <= thresh) && (0 >= x); // x shows negative values in headed direction...
    // return (sqrt(pow(z,2) + pow(y,2)) <= thresh) && (0 >= x); // x shows negative values in headed direction...
}

bool isValidTranslation_cam_fr(cv::Mat t, double thresh = 0.5)
{
    double x = t.at<double>(0);
    double y = t.at<double>(1);
    double z = t.at<double>(2);

    return (-thresh <= z && z <= thresh) && (0 <= x); // x shows negative values in headed direction...
}


// void correctInvalidRotation(cv::Mat& R)
// {
//     cv::Mat euler = cv::Mat::zeros(3,1, CV_64F);
//     cv::Rodrigues(R, euler);
    
//     double phi = euler.at<double>(0);
//     double theta = euler.at<double>(1);
//     double psi = euler.at<double>(2);

//     // if (phi > M_PI)
//     //     phi -= M_PI;
//     // if (phi > M_PI/2)
//     //     phi -= M_PI/2;
//     // if (phi < -M_PI)
//     //     phi += M_PI;
//     // if (phi < -M_PI/2)
//     //     phi += M_PI/2;

//     // if (theta > M_PI)
//     //     theta -= M_PI;
//     // if (theta > M_PI/2)
//     //     theta -= M_PI/2;
//     // if (theta < -M_PI)
//     //     theta += M_PI;
//     // if (theta < -M_PI/2)
//     //     theta += M_PI/2;

//     // if (psi > M_PI)
//     //     psi -= M_PI;
//     // if (psi > M_PI/2)
//     //     psi -= M_PI/2;
//     // if (psi < -M_PI)
//     //     psi += M_PI;
//     // if (psi < -M_PI/2)
//     //     psi += M_PI/2;

//     // if (phi > (3*M_PI/4))
//     //     phi -= M_PI;
//     // if (phi > (3*M_PI/4)/2)
//     //     phi -= M_PI/2;
//     // if (phi < -(3*M_PI/4))
//     //     phi += M_PI;
//     // if (phi < -(3*M_PI/4)/2)
//     //     phi += M_PI/2;

//     // if (theta > (3*M_PI/4))
//     //     theta -= M_PI;
//     // if (theta > (3*M_PI/4)/2)
//     //     theta -= M_PI/2;
//     // if (theta < -(3*M_PI/4))
//     //     theta += M_PI;
//     // if (theta < -(3*M_PI/4)/2)
//     //     theta += M_PI/2;

//     // if (psi > (3*M_PI/4))
//     //     psi -= M_PI;
//     // if (psi > (3*M_PI/4)/2)
//     //     psi -= M_PI/2;
//     // if (psi < -(3*M_PI/4))
//     //     psi += M_PI;
//     // if (psi < -(3*M_PI/4)/2)
//     //     psi += M_PI/2;
    
//     if (phi > M_PI || phi < -M_PI)
//         phi = std::fmod(phi,M_PI);
//     if (phi > M_PI/2 || phi < -M_PI/2)
//         phi = std::fmod(phi,M_PI/2);
    
//     if (theta > M_PI || theta < -M_PI)
//         theta = std::fmod(theta,M_PI);
//     if (theta > M_PI/2 || theta < -M_PI/2)
//         theta = std::fmod(theta,M_PI/2);

//     if (psi > M_PI || psi < -M_PI)
//         psi = std::fmod(psi,M_PI);
//     if (psi > M_PI/2 || psi < -M_PI/2)
//         psi = std::fmod(psi,M_PI/2);


//     // if (phi > (3*M_PI/4) || phi < -(3*M_PI/4))
//     //     phi = std::fmod(phi,(3*M_PI/4));
//     // if (phi > (3*M_PI/4)/2 || phi < -(3*M_PI/4)/2)
//     //     phi = std::fmod(phi,(3*M_PI/4)/2);
    
//     // if (theta > (3*M_PI/4) || theta < -(3*M_PI/4))
//     //     theta = std::fmod(theta,(3*M_PI/4));
//     // if (theta > (3*M_PI/4)/2 || theta < -(3*M_PI/4)/2)
//     //     theta = std::fmod(theta,(3*M_PI/4)/2);

//     // if (psi > (3*M_PI/4) || psi < -(3*M_PI/4))
//     //     psi = std::fmod(psi,(3*M_PI/4));
//     // if (psi > (3*M_PI/4)/2 || psi < -(3*M_PI/4)/2)
//     //     psi = std::fmod(psi,(3*M_PI/4)/2);

//     cv::Mat euler_cor = (cv::Mat_<double>(3,1) << phi, 
// 												theta, 
// 												psi);

//     cv::Mat R_r;
// 	cv::Rodrigues(euler_cor, R_r);
//     R = R_r;	
// }

cv::Mat correctInvalidRotation(cv::Mat R)
{
    cv::Mat euler = cv::Mat::zeros(3,1, CV_64F);
    cv::Rodrigues(R, euler);
    
    double phi = euler.at<double>(0);
    double theta = euler.at<double>(1);
    double psi = euler.at<double>(2);

    // if (phi > M_PI)
    //     phi -= M_PI;
    // if (phi > M_PI/2)
    //     phi -= M_PI/2;
    // if (phi < -M_PI)
    //     phi += M_PI;
    // if (phi < -M_PI/2)
    //     phi += M_PI/2;

    // if (theta > M_PI)
    //     theta -= M_PI;
    // if (theta > M_PI/2)
    //     theta -= M_PI/2;
    // if (theta < -M_PI)
    //     theta += M_PI;
    // if (theta < -M_PI/2)
    //     theta += M_PI/2;

    // if (psi > M_PI)
    //     psi -= M_PI;
    // if (psi > M_PI/2)
    //     psi -= M_PI/2;
    // if (psi < -M_PI)
    //     psi += M_PI;
    // if (psi < -M_PI/2)
    //     psi += M_PI/2;
    
    if (phi > M_PI || phi < -M_PI)
        phi = std::fmod(phi,M_PI);
    if (phi > M_PI/2 || phi < -M_PI/2)
        phi = std::fmod(phi,M_PI/2);
    
    if (theta > M_PI || theta < -M_PI)
    {
        ROS_INFO("THETA LARGER \n\n");
        theta = std::fmod(theta,M_PI);

    }
    if (theta > M_PI/2 || theta < -M_PI/2)
        theta = std::fmod(theta,M_PI/2);

    if (psi > M_PI || psi < -M_PI)
        psi = std::fmod(psi,M_PI);
    if (psi > M_PI/2 || psi < -M_PI/2)
        psi = std::fmod(psi,M_PI/2);


    cv::Mat euler_cor = (cv::Mat_<double>(3,1) << phi, 
												theta, 
												psi);

    cv::Mat R_r;
	cv::Rodrigues(euler_cor, R_r);
    return R_r;
}

void correctInvalidTranslation(cv::Mat& t, cv::Mat t_gt, double thresh)
{
    double x = t.at<double>(0);
    double y = t.at<double>(1);
    double z = t.at<double>(2);
    
    if ( (sqrt(pow(z,2) + pow(y,2)) >= thresh) || (0 <= x) ); // x shows negative values in headed direction...
        t = 1*t_gt + 0*t;
}

void correctInvalidTranslation(cv::Mat& t, double thresh)
{
    double x = t.at<double>(0);
    double y = t.at<double>(1);
    double z = t.at<double>(2);
    
    if ( (sqrt(pow(z,2) + pow(y,2)) >= thresh) ); // x shows negative values in headed direction...
    {
        ROS_INFO("Transform");
        cv::Mat t_cor = (cv::Mat_<double>(3,1) << y, 
												x, 
												z);
        t = t_cor;
    }
}


// Calculates rotation matrix given euler angles.
cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f &theta)
{
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );
    
    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );
    
    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);
    
    
    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;
    
    return R;

}


// Calculates rotation matrix given euler angles.
cv::Mat rotateZ(double psi)
{
    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
                cos(psi),     -sin(psi),       0,
                sin(psi),     cos(psi),        0,
                0,               0,                  1);
    
    return R_z;
}




void cv2body_ma2(cv::Mat R_c1c2, cv::Mat t_c1c2, cv::Mat& R_b1b2, cv::Mat& t_b1b2)
{
    cv::Mat T_c1c2 = (cv::Mat_<double>(4,4) <<
                R_c1c2.at<double>(0,0), R_c1c2.at<double>(0,1), R_c1c2.at<double>(0,2), t_c1c2.at<double>(0),
                R_c1c2.at<double>(1,0), R_c1c2.at<double>(1,1), R_c1c2.at<double>(1,2), t_c1c2.at<double>(1),
                R_c1c2.at<double>(2,0), R_c1c2.at<double>(2,1), R_c1c2.at<double>(2,2), t_c1c2.at<double>(2),
                                            0,               			 0,                  		0, 						  1);

    cv::Mat T_bc = (cv::Mat_<double>(4,4) <<  
                    0, 0, 1, 0,
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 0, 1);

    cv::Mat T_cb = T_bc.t();

    cv::Mat T_b1b2 = T_bc * T_c1c2 * T_cb;

    R_b1b2 = (cv::Mat_<double>(3,3) <<
                T_b1b2.at<double>(0,0), T_b1b2.at<double>(0,1), T_b1b2.at<double>(0,2),
                T_b1b2.at<double>(1,0), T_b1b2.at<double>(1,1), T_b1b2.at<double>(1,2),
                T_b1b2.at<double>(2,0), T_b1b2.at<double>(2,1), T_b1b2.at<double>(2,2));
    
    t_b1b2 = (cv::Mat_<double>(3,1) <<
                T_b1b2.at<double>(0,3),
                T_b1b2.at<double>(1,3),
                T_b1b2.at<double>(2,3));
}



void cv2body_ma2_FL(cv::Mat R_c1c2, cv::Mat t_c1c2, cv::Mat& R_b1b2, cv::Mat& t_b1b2, cv::Mat R_cf_cfl)
{
    // In body frame
    cv::Mat T_fl_b = (cv::Mat_<double>(4,4) <<
                R_cf_cfl.at<double>(0,0), R_cf_cfl.at<double>(0,1), R_cf_cfl.at<double>(0,2), 0,
                R_cf_cfl.at<double>(1,0), R_cf_cfl.at<double>(1,1), R_cf_cfl.at<double>(1,2), 0,
                R_cf_cfl.at<double>(2,0), R_cf_cfl.at<double>(2,1), R_cf_cfl.at<double>(2,2), 0,
                                       0,               			 0,                  		0, 						  1);
    
    cv::Mat T_b_fl = T_fl_b.t();

    // In OpenCV frame
    cv::Mat T_c1c2 = (cv::Mat_<double>(4,4) <<
                R_c1c2.at<double>(0,0), R_c1c2.at<double>(0,1), R_c1c2.at<double>(0,2), t_c1c2.at<double>(0),
                R_c1c2.at<double>(1,0), R_c1c2.at<double>(1,1), R_c1c2.at<double>(1,2), t_c1c2.at<double>(1),
                R_c1c2.at<double>(2,0), R_c1c2.at<double>(2,1), R_c1c2.at<double>(2,2), t_c1c2.at<double>(2),
                                            0,               			 0,                  		0, 						  1);

    // OpenCV to body frame
    cv::Mat T_bc = (cv::Mat_<double>(4,4) <<  
                    0, 0, 1, 0,
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 0, 1);

    cv::Mat T_cb = T_bc.t();

    cv::Mat T_b1b2 = T_b_fl * T_bc * T_c1c2 * T_cb * T_fl_b ;

    R_b1b2 = (cv::Mat_<double>(3,3) <<
                T_b1b2.at<double>(0,0), T_b1b2.at<double>(0,1), T_b1b2.at<double>(0,2),
                T_b1b2.at<double>(1,0), T_b1b2.at<double>(1,1), T_b1b2.at<double>(1,2),
                T_b1b2.at<double>(2,0), T_b1b2.at<double>(2,1), T_b1b2.at<double>(2,2));
    
    t_b1b2 = (cv::Mat_<double>(3,1) <<
                T_b1b2.at<double>(0,3),
                T_b1b2.at<double>(1,3),
                T_b1b2.at<double>(2,3));
}

void cv2body_ma2_FR(cv::Mat R_c1c2, cv::Mat t_c1c2, cv::Mat& R_b1b2, cv::Mat& t_b1b2, cv::Mat R_cf_cfl)
{
    // In body frame
    cv::Mat T_fl_b = (cv::Mat_<double>(4,4) <<
                R_cf_cfl.at<double>(0,0), R_cf_cfl.at<double>(0,1), R_cf_cfl.at<double>(0,2), 0,
                R_cf_cfl.at<double>(1,0), R_cf_cfl.at<double>(1,1), R_cf_cfl.at<double>(1,2), 0,
                R_cf_cfl.at<double>(2,0), R_cf_cfl.at<double>(2,1), R_cf_cfl.at<double>(2,2), 0,
                                       0,               			 0,                  		0, 						  1);
    
    cv::Mat T_b_fl = T_fl_b.t();

    // In OpenCV frame
    cv::Mat T_c1c2 = (cv::Mat_<double>(4,4) <<
                R_c1c2.at<double>(0,0), R_c1c2.at<double>(0,1), R_c1c2.at<double>(0,2), t_c1c2.at<double>(0),
                R_c1c2.at<double>(1,0), R_c1c2.at<double>(1,1), R_c1c2.at<double>(1,2), t_c1c2.at<double>(1),
                R_c1c2.at<double>(2,0), R_c1c2.at<double>(2,1), R_c1c2.at<double>(2,2), t_c1c2.at<double>(2),
                                            0,               			 0,                  		0, 						  1);

    // OpenCV to body frame
    cv::Mat T_bc = (cv::Mat_<double>(4,4) <<  
                    0, 0, 1, 0,
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 0, 1);

    cv::Mat T_cb = T_bc.t();

    cv::Mat T_b1b2 = T_b_fl * T_bc * T_c1c2 * T_cb * T_fl_b ;

    R_b1b2 = (cv::Mat_<double>(3,3) <<
                T_b1b2.at<double>(0,0), T_b1b2.at<double>(0,1), T_b1b2.at<double>(0,2),
                T_b1b2.at<double>(1,0), T_b1b2.at<double>(1,1), T_b1b2.at<double>(1,2),
                T_b1b2.at<double>(2,0), T_b1b2.at<double>(2,1), T_b1b2.at<double>(2,2));
    
    t_b1b2 = (cv::Mat_<double>(3,1) <<
                T_b1b2.at<double>(0,3),
                T_b1b2.at<double>(1,3),
                T_b1b2.at<double>(2,3));
}



void cv2body_kitti(cv::Mat R_c1c2, cv::Mat t_c1c2, cv::Mat& R_b1b2, cv::Mat& t_b1b2)
{
    cv::Mat T_c1c2 = (cv::Mat_<double>(4,4) <<
                R_c1c2.at<double>(0,0), R_c1c2.at<double>(0,1), R_c1c2.at<double>(0,2), t_c1c2.at<double>(0),
                R_c1c2.at<double>(1,0), R_c1c2.at<double>(1,1), R_c1c2.at<double>(1,2), t_c1c2.at<double>(1),
                R_c1c2.at<double>(2,0), R_c1c2.at<double>(2,1), R_c1c2.at<double>(2,2), t_c1c2.at<double>(2),
                                            0,               			 0,                  		0, 						  1);

    cv::Mat T_bc = (cv::Mat_<double>(4,4) <<  
                     1,  0,  0, 0,
                     0, -1,  0, 0,
                     0,  0, -1, 0,
                     0,  0,  0, 1);

    // cv::Mat T_bc = (cv::Mat_<double>(4,4) <<  
    //                 1,  0, 0, 0,
    //                 0,  0, 1, 0,
    //                 0, -1, 0, 0,
    //                 0,  0, 0, 1);

    cv::Mat T_cb = T_bc.t();

    cv::Mat T_b1b2 = T_bc * T_c1c2 * T_cb;

    R_b1b2 = (cv::Mat_<double>(3,3) <<
                T_b1b2.at<double>(0,0), T_b1b2.at<double>(0,1), T_b1b2.at<double>(0,2),
                T_b1b2.at<double>(1,0), T_b1b2.at<double>(1,1), T_b1b2.at<double>(1,2),
                T_b1b2.at<double>(2,0), T_b1b2.at<double>(2,1), T_b1b2.at<double>(2,2));
    
    t_b1b2 = (cv::Mat_<double>(3,1) <<
                T_b1b2.at<double>(0,3),
                T_b1b2.at<double>(1,3),
                T_b1b2.at<double>(2,3));
}