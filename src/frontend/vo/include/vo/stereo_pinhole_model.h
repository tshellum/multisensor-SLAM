#pragma once

#include <string>
#include <iostream>

#include <Eigen/Dense> 

#include "opencv2/core/eigen.hpp"
#include <opencv2/calib3d.hpp>

#include <boost/property_tree/ptree.hpp>


struct PinholeModel
{
  // Distortion parameters  
  cv::Mat distortion;
    
  // Intrinsics
  cv::Mat K_cv;
  Eigen::Matrix3d K_eig; 

  // Projection matrix
  cv::Mat P;

  PinholeModel(std::string placement, boost::property_tree::ptree params)
  {
    distortion = cv::Mat_<double>(5,1);
    double intrinsics[9]; 
    double proj_params[12]; 

    try
    {
      int i = 0;
      BOOST_FOREACH(boost::property_tree::ptree::value_type &v, params.get_child("camera_" + placement + ".distortion"))
        distortion.at<double>(i++) = v.second.get_value<double>(); 

      i = 0;
      BOOST_FOREACH(boost::property_tree::ptree::value_type &v, params.get_child("camera_" + placement + ".intrinsics"))
        intrinsics[i++] = v.second.get_value<double>(); 

      i = 0;
      BOOST_FOREACH(boost::property_tree::ptree::value_type &v, params.get_child("camera_" + placement + ".projection_matrix"))
        proj_params[i++] = v.second.get_value<double>(); 
    }
    catch (std::exception const& e)
    {
        std::cerr << e.what() << std::endl;
    }

    K_cv = (cv::Mat_<double>(3,3) << 
      intrinsics[0], intrinsics[1], intrinsics[2],
      intrinsics[3], intrinsics[4], intrinsics[5],
      intrinsics[6], intrinsics[7], intrinsics[8]);

    P = (cv::Mat_<double>(3,4) << 
      proj_params[0], proj_params[1],  proj_params[2],  proj_params[3],
      proj_params[4], proj_params[5],  proj_params[6],  proj_params[7],
      proj_params[8], proj_params[9], proj_params[10], proj_params[11]);

    // K_cv = cv::Mat(3, 3, CV_64F, intrinsics);
    // P    = cv::Mat(3, 4, CV_64F, proj_params);

    cv::cv2eigen(K_cv, K_eig);
  }
};

// -------------------------------------------------------------------

class StereoPinholeModel
{
private:
  // Image parameters  
  int width_, height_;
  int crop_width_, crop_height_;

  PinholeModel camera_left_;
  PinholeModel camera_right_;

  // Tranformation between cameras 
  Eigen::Affine3d T_clcr_, T_crcl_;

public:
  StereoPinholeModel(boost::property_tree::ptree camera_params,
                     boost::property_tree::ptree detector_params)
  : width_( camera_params.get< int >("image_width") )
  , height_( camera_params.get< int >("image_height") )
  , camera_left_("left", camera_params)
  , camera_right_("right", camera_params)
  , T_clcr_(Eigen::Affine3d::Identity())
  , T_crcl_(Eigen::Affine3d::Identity())
  {
    int patch_size = detector_params.get< int >("detector.grid_size");
    crop_width_ = width_ - (width_ % patch_size);
		crop_height_ = height_ - (height_ % patch_size);

    double t_params[3]; 
    double R_params[9]; 

    try
    {
      int i = 0;
      BOOST_FOREACH(boost::property_tree::ptree::value_type &v, camera_params.get_child("stereo.rotation"))
        R_params[i++] = v.second.get_value<double>(); 

      i = 0;
      BOOST_FOREACH(boost::property_tree::ptree::value_type &v, camera_params.get_child("stereo.translation"))
        t_params[i++] = v.second.get_value<double>(); 
    }
    catch (std::exception const& e)
    {
        std::cerr << e.what() << std::endl;
    }

    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > R(R_params);
    Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor> > t(t_params);
    T_crcl_.linear() = R;
    T_crcl_.translation() = t;

    T_clcr_.linear() = R.transpose();
    T_clcr_.translation() = -t;
  }

  ~StereoPinholeModel()
  {}

  int getImageWidth()  {return crop_width_;};
  int getImageHeight() {return crop_height_;};

  PinholeModel l() {return camera_left_;};
	PinholeModel r() {return camera_right_;};

	cv::Mat lProjMat() {return camera_left_.P;};
	cv::Mat rProjMat() {return camera_right_.P;};
	Eigen::Affine3d T_lr() {return T_clcr_;};
	Eigen::Affine3d T_rl() {return T_crcl_;};

  cv::Mat K_cv() {return camera_left_.K_cv;};
  Eigen::Matrix3d K_eig() {return camera_left_.K_eig;};

	std::pair<cv::Mat, cv::Mat> preprocessImagePair(cv::Mat img_left, cv::Mat img_right);
};


std::pair<cv::Mat, cv::Mat> StereoPinholeModel::preprocessImagePair(cv::Mat img_left, 
															                                      cv::Mat img_right)
{
  cv::Mat limg_edit, rimg_edit;
  cv::undistort(img_left, limg_edit, camera_left_.K_cv, camera_left_.distortion, cv::Mat());
  cv::undistort(img_right, rimg_edit, camera_right_.K_cv, camera_right_.distortion, cv::Mat());

  limg_edit = limg_edit(cv::Rect(0, 0, crop_width_, crop_height_)).clone();
  rimg_edit = rimg_edit(cv::Rect(0, 0, crop_width_, crop_height_)).clone();

	return std::make_pair(limg_edit, rimg_edit);
}