#pragma once


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

    void triangulate(std::vector<cv::KeyPoint> match_left, 
                    std::vector<cv::KeyPoint> match_right,
                    cv::Mat K_l, 
                    cv::Mat K_r,
                    cv::Mat R_clcr,
                    cv::Mat t_clcr);
};


void PointCloudManager::setExtrinsics(Eigen::Matrix3d R_wb, Eigen::Vector3d t_wb)
{
    _pc.R_wb = R_wb;
    _pc.t_wb = t_wb;
}



void PointCloudManager::triangulate(std::vector<cv::KeyPoint> match_left, 
                                    std::vector<cv::KeyPoint> match_right,
                                    cv::Mat K_l, 
                                    cv::Mat K_r,
                                    cv::Mat R_clcr,
                                    cv::Mat t_clcr)
{
    std::vector<cv::Point2f> point2D_left, point2D_right;
    cv::KeyPoint::convert(match_left, point2D_left);
    cv::KeyPoint::convert(match_right, point2D_right);

    cv::Mat Rt_l = cv::Mat::eye(3, 4, CV_64FC1);
    cv::Mat Rt_r = cv::Mat::eye(3, 4, CV_64FC1);
    cv::hconcat(R_clcr, t_clcr, Rt_r);
    
    // cv::Mat P_l, P_r;
    // cv::stereoRectify(K_l, cv::Mat(),
    //                   K_r, cv::Mat(),

		// 									cv::Mat(),
		// 									cv::Mat(),
		// 									R_clcr,
		// 									t_clcr);

    // cv::stereoRectify	(	InputArray 	cameraMatrix1,
    //                         InputArray 	distCoeffs1,
    //                         InputArray 	cameraMatrix2,
    //                         InputArray 	distCoeffs2,
    //                         Size 	imageSize,
    //                         InputArray 	R,
    //                         InputArray 	T,
    //                         OutputArray 	R1,
    //                         OutputArray 	R2,
    //                         OutputArray 	P1,
    //                         OutputArray 	P2,
    //                         OutputArray 	Q,
    //                         int 	flags = CALIB_ZERO_DISPARITY,
    //                         double 	alpha = -1,
    //                         Size 	newImageSize = Size(),
    //                         Rect * 	validPixROI1 = 0,
    //                         Rect * 	validPixROI2 = 0 
    //                         )		


    
    int N = point2D_left.size();
    cv::Mat point3D_homogenous(4, N, CV_64FC1);                                                 // https://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints
    cv::triangulatePoints(K_l*Rt_l, K_r*Rt_r, point2D_left, point2D_right, point3D_homogenous); // https://gist.github.com/cashiwamochi/8ac3f8bab9bf00e247a01f63075fedeb

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
        
        ROS_INFO_STREAM("wp: " << wp);
    }
    
    _pc.world_points = point3D_cartesian;    
}
