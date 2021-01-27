#pragma once


/*** OpenGV packages ***/
#include "opengv/include/triangulation/methods.hpp"

struct PointCloud 
{
    std::vector<cv::Point3f> world_points;
    Eigen::Matrix3d R_wb;
    Eigen::Vector3d t_wb;
    int id;
};


class PointCloudManager
{
private:
    PointCloud _pc;
public:
    PointCloudManager(){};
    ~PointCloudManager(){};

    void setExtrinsics(Eigen::Matrix3d R_wb, Eigen::Vector3d t_wb);

    void keypoint2bearing(std::vector<cv::KeyPoint> match_left, 
													std::vector<cv::KeyPoint> match_right,
													Eigen::Matrix3d K_l,
													Eigen::Matrix3d K_r,
													opengv::bearingVectors_t& bearing_l, 
													opengv::bearingVectors_t& bearing_r);

    void triangulate(std::vector<cv::KeyPoint> match_left, 
                    std::vector<cv::KeyPoint> match_right,
                    cv::Mat P_l, 
                    cv::Mat P_r,
										cv::Mat K);
};


void PointCloudManager::setExtrinsics(Eigen::Matrix3d R_wb, Eigen::Vector3d t_wb)
{
    _pc.R_wb = R_wb;
    _pc.t_wb = t_wb;
}



void PointCloudManager::keypoint2bearing(std::vector<cv::KeyPoint> match_left, 
																				std::vector<cv::KeyPoint> match_right,
																				Eigen::Matrix3d K_l,
																				Eigen::Matrix3d K_r,
																				opengv::bearingVectors_t& bearing_l, 
																				opengv::bearingVectors_t& bearing_r)
{

	for (int i = 0; i < match_left.size(); i++) 
	{
		// match_left[i].pt.x;
	} 

	// (pixel.x - imageCentreX, pixel.y - imageCentreY, focalLength);
	// rotation * (pixel.x - imageCentreX, pixel.y - imageCentreY, focalLength);

}



// void PointCloudManager::triangulate(std::vector<cv::KeyPoint> match_left, 
//                                     std::vector<cv::KeyPoint> match_right,
//                                     cv::Mat P_l, 
//                                     cv::Mat P_r)
// {
    // https://gist.github.com/edgarriba/10246de35799d63e0423
		// https://stackoverflow.com/questions/49580137/comparing-opencv-pnp-with-opengv-pnp
		// https://stackoverflow.com/questions/41225420/get-bearing-vector-direction-from-pixel-in-image
// }



void PointCloudManager::triangulate(std::vector<cv::KeyPoint> match_left, 
                                    std::vector<cv::KeyPoint> match_right,
                                    cv::Mat P_l, 
                                    cv::Mat P_r,
																		cv::Mat K)
{
    std::vector<cv::Point2f> point2D_left, point2D_right;
    cv::KeyPoint::convert(match_left, point2D_left);
    cv::KeyPoint::convert(match_right, point2D_right);

		cv::Mat E, inliers;
		E = cv::findEssentialMat(point2D_left, point2D_right, K, cv::RANSAC, 0.999, 1.0, inliers); 

		cv::Mat R_c1c2, t_c1c2;
		cv::recoverPose(E, point2D_left, point2D_right, K, R_c1c2, t_c1c2); // z = viewer direction, x and y follows camera frame

		cv::Mat Rt_l = cv::Mat::eye(3, 4, CV_64FC1);
		cv::Mat Rt_r = cv::Mat::eye(3, 4, CV_64FC1);
		cv::hconcat(R_c1c2, t_c1c2, Rt_r);


		ROS_INFO_STREAM("Left to right, R_c1c2: " << R_c1c2);
		ROS_INFO_STREAM("Left to right, t_c1c2: " << t_c1c2);

    int N = point2D_left.size();
    cv::Mat point3D_homogenous(4, N, CV_64FC1);                                                 // https://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints
		cv::triangulatePoints(K*Rt_l, K*Rt_r, point2D_left, point2D_right, point3D_homogenous); // https://gist.github.com/cashiwamochi/8ac3f8bab9bf00e247a01f63075fedeb

    // cv::triangulatePoints(P_l, P_r, point2D_left, point2D_right, point3D_homogenous); // https://gist.github.com/cashiwamochi/8ac3f8bab9bf00e247a01f63075fedeb

    std::vector<cv::Point3f> point3D_cartesian;
    for(int i = 0; i < N; i++) 
    {
        cv::Point3f wp;
        cv::Mat p3c;
        cv::Mat p3h = point3D_homogenous.col(i);
        // ROS_INFO_STREAM("p3h: " << p3h);

        cv::convertPointsFromHomogeneous(p3h.t(), p3c);
        // ROS_INFO_STREAM("p3c: " << p3c);

        wp.x = p3c.at<float>(0);
        wp.y = p3c.at<float>(1);
        wp.z = p3c.at<float>(2);

        point3D_cartesian.push_back(wp);
        
        // if (i % 10 == 0)
				ROS_INFO_STREAM("wp: " << wp);
    }
    
    _pc.world_points = point3D_cartesian;    
}
