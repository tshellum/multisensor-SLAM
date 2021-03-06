#pragma once

#include <string>
#include <iostream>

#include <Eigen/Dense> 

#include "opencv2/core/eigen.hpp"
#include <opencv2/calib3d.hpp>

#include <boost/property_tree/ptree.hpp>

#include "inputOutput.h"

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

    cv::cv2eigen(K_cv, K_eig);

    P = (cv::Mat_<double>(3,4) << 
      proj_params[0], proj_params[1],  proj_params[2],  proj_params[3],
      proj_params[4], proj_params[5],  proj_params[6],  proj_params[7],
      proj_params[8], proj_params[9], proj_params[10], proj_params[11]);


    // if (placement == "right")
    //   P.at<double>(0, 3) = -2152.4398853199;

    // std::cout << placement << " P: \n" << P << std::endl;

    // double t_params[3]; 
    // double R_params[9]; 

    // try
    // {
    //   int i = 0;
    //   BOOST_FOREACH(boost::property_tree::ptree::value_type &v, params.get_child("stereo.rotation"))
    //     R_params[i++] = v.second.get_value<double>(); 

    //   i = 0;
    //   BOOST_FOREACH(boost::property_tree::ptree::value_type &v, params.get_child("stereo.translation"))
    //     t_params[i++] = v.second.get_value<double>(); 
    // }
    // catch (std::exception const& e)
    // {
    //     std::cerr << e.what() << std::endl;
    // }


    // P = cv::Mat::eye(cv::Size(4,3), CV_64F);

    // if (placement == "right")
    // {
    //   cv::Mat R(3, 3, CV_64F, R_params);
    //   cv::Mat t(3, 1, CV_64F, t_params);
  
    //   R.copyTo(P(cv::Rect(0, 0, 3, 3))); 
    //   t.copyTo(P(cv::Rect(3, 0, 1, 3)));
    // }

    // std::cout << placement << " P: \n" << P << std::endl;

    // P = K_cv * P;
    // std::cout << placement << " P: \n" << P << std::endl;

// 1166.63168894294, -7.20692985562, 762.63559915158, -2096.0786956901;
//  -35.39667059034, 1243.82082546096, 512.9316476381, 34.4586267311;
//  -0.09935919999999999, 0.0132798, 0.994963, 0.087338

    // K_cv = cv::Mat(3, 3, CV_64F, intrinsics).clone();
    // P    = cv::Mat(3, 4, CV_64F, proj_params).clone();
  }
};

// -------------------------------------------------------------------

class StereoPinholeModel
{
private:
  // Brightness parameter
  bool brightness_gamma_correction_;
  double brightness_gain_;

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
    brightness_gamma_correction_ = camera_params.get< bool >("preprocess.brightness_gamma_correction");
    brightness_gain_ = camera_params.get< double >("preprocess.brightness_gain");

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
  cv::Mat limg_undist, rimg_undist;
  cv::undistort(img_left, limg_undist, camera_left_.K_cv, camera_left_.distortion, cv::Mat());
  cv::undistort(img_right, rimg_undist, camera_right_.K_cv, camera_right_.distortion, cv::Mat());

  cv::Mat lcropped = limg_undist(cv::Rect(x0_, y0_, crop_width_, crop_height_));
  cv::Mat rcropped = rimg_undist(cv::Rect(x0_, y0_, crop_width_, crop_height_));

  // Gamma correction of brightness
  if ( (brightness_gain_ >= 0) && (brightness_gain_ != 1) )
  {
    if (brightness_gamma_correction_)
    {
      for( int row = 0; row < crop_height_; row++)
      {
        for( int col = 0; col < crop_width_; col++)
        {
            lcropped.at<uchar>(row,col) = cv::saturate_cast<uchar>(pow(lcropped.at<uchar>(row,col) / 255.0, brightness_gain_) * 255.0);
            rcropped.at<uchar>(row,col) = cv::saturate_cast<uchar>(pow(rcropped.at<uchar>(row,col) / 255.0, brightness_gain_) * 255.0);
        }
      }
    }
    else
    {
      lcropped *= brightness_gain_;
      rcropped *= brightness_gain_;
    }
  }

	return std::make_pair(lcropped, rcropped);
}