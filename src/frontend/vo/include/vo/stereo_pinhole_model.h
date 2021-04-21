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
  // Brightness parameter
  double brightness_gamma_correction_;

  // Image parameters  
  int width_, height_;
  int x0_, y0_;
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
    brightness_gamma_correction_ = camera_params.get< double >("preprocess.brightness_gamma_correction");

    int patch_size = detector_params.get< int >("detector.grid_size");
    x0_ = std::max(0, camera_params.get< int >("preprocess.crop.x0"));
    y0_ = std::max(0, camera_params.get< int >("preprocess.crop.y0"));

    int crop_width  = camera_params.get< int >("preprocess.crop.width");
    if ( (crop_width <= 0) || (crop_width > width_ - x0_) )
      crop_width = width_ - x0_;
    
    int crop_height = camera_params.get< int >("preprocess.crop.height");
    if ( (crop_height <= 0) || (crop_height > height_ - y0_) )
      crop_height = height_ - y0_;

    crop_width_ = crop_width - (crop_width % patch_size);
		crop_height_ = crop_height - (crop_height % patch_size);

    std::cout << "Image dimentions is set to: [" << crop_width_ << " x " << crop_height_ << "]" << std::endl;

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

  int getOriginalImageWidth()  {return width_;};
  int getOriginalImageHeight() {return height_;};
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

  cv::Mat lcropped = limg_edit(cv::Rect(x0_, y0_, crop_width_, crop_height_));
  cv::Mat rcropped = rimg_edit(cv::Rect(x0_, y0_, crop_width_, crop_height_));

  // Gamma correction of brightness
  if ( (brightness_gamma_correction_ >= 0) && (brightness_gamma_correction_ != 1) )
  {
    for( int row = 0; row < crop_height_; row++)
    {
      for( int col = 0; col < crop_width_; col++)
      {
          lcropped.at<uchar>(row,col) = cv::saturate_cast<uchar>(pow(lcropped.at<uchar>(row,col) / 255.0, brightness_gamma_correction_) * 255.0);
          rcropped.at<uchar>(row,col) = cv::saturate_cast<uchar>(pow(rcropped.at<uchar>(row,col) / 255.0, brightness_gamma_correction_) * 255.0);
      }
    }
  }
  // lcropped *= 1.2;
  // rcropped *= 1.2;

	return std::make_pair(lcropped, rcropped);
}