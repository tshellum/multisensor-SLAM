#pragma once

#include <ros/package.h>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <opencv2/opencv.hpp>

#include <Eigen/Geometry> 

#include <sys/stat.h> 
#include <iostream>
#include <iomanip>


void NotImplementedError(std::string function_name, std::string filename) // call by NotImplementedError(__func__, __FILE__);
{
  std::vector<std::string> entries;
  boost::split(entries, filename, boost::is_any_of("/"));

  ROS_ERROR_STREAM("Function " << function_name << "() in " << entries.rbegin()[1] << "/" << entries.rbegin()[0] << " not implemented.");
}


boost::property_tree::ptree readConfigFromJsonFile(const std::string& filename) 
{
	boost::property_tree::ptree pt;

	try {
		boost::property_tree::read_json(filename, pt);
	} catch (std::exception& e) {
		std::cerr << "Error Reading File " << filename << ":" << std::endl;
		std::cerr << e.what() << std::endl;
		throw;
	}

	return pt;
}


void displayWindow
(
  cv::Mat image1, cv::Mat image2=cv::Mat(), 
  std::string name="Stereo images", int resizeWidth=1920, int resizeHeight=1080, int key=3
)
{
  if (image1.empty() && image2.empty())
    return;

  // Decide image to be displayed
  cv::Mat display_image;
  if (image2.empty())
    display_image = image1;

  else if (image1.empty())
    display_image = image2;
  
  else
    cv::hconcat(image1, image2, display_image);

  // Resize
  double width_ratio = (double) display_image.cols / resizeWidth;
  double height_ratio = (double) display_image.rows / resizeHeight;
  double ratio = std::max(width_ratio, height_ratio);
  resize(display_image, display_image, cv::Size(display_image.cols / ratio, display_image.rows / ratio)); 

  // Display
  cv::namedWindow(name, cv::WINDOW_FULLSCREEN);
  cv::imshow(name, display_image);
  cv::waitKey(key);
}


void displayWindowFeatures
(
  cv::Mat image1, std::vector<cv::KeyPoint> kps1={}, 
  cv::Mat image2=cv::Mat(), std::vector<cv::KeyPoint> kps2={},
  std::string name="Detections", int resizeWidth=1920, int resizeHeight=1080, int key=3
)
{
  cv::Mat img_kps1 = image1; cv::Mat img_kps2 = image2;
  
  if (! kps1.empty())
    cv::drawKeypoints(image1, kps1, img_kps1, cv::Scalar(0, 255, 255), cv::DrawMatchesFlags::DEFAULT);
    
  if (! kps2.empty())
    cv::drawKeypoints(image2, kps2, img_kps2, cv::Scalar(0, 255, 255), cv::DrawMatchesFlags::DEFAULT);
  
  displayWindow(img_kps1, img_kps2, name, resizeWidth, resizeHeight, key);
}


void displayWindowMatches
(
  cv::Mat image1, std::vector<cv::KeyPoint> kps1, 
  cv::Mat image2, std::vector<cv::KeyPoint> kps2,
  std::vector<cv::DMatch> matches,
  std::string name="Matches", int resizeWidth=1920, int resizeHeight=1080, int key=3
)
{
  cv::Mat im_match; 
  cv::drawMatches(image1, kps1, image2, kps2, matches, im_match); 
  displayWindow(im_match, cv::Mat(), name, resizeWidth, resizeHeight, key);
}


cv::Mat drawPoints(cv::Mat image, std::vector<cv::Point2f> points)
{
    int r = 3;
    for (int i=0;i<points.size();i++)
        circle(image, cvPoint(points[i].x, points[i].y), r, CV_RGB(255,255,0), 1, 8, 0);

    return image;    
}


cv::Mat drawEpiLines(cv::Mat image, cv::Mat F, int LeftRight, std::vector<cv::Point2f> points)
{
    std::vector<cv::Vec3f> epiLines;
    cv::computeCorrespondEpilines(points, LeftRight, F, epiLines);

    // for all epipolar lines
    for (auto it = epiLines.begin(); it != epiLines.end(); ++it)
    {
        // draw the epipolar line between first and last column
        cv::line(image, cv::Point(0, -(*it)[2] / (*it)[1]),
                cv::Point(image.cols, -((*it)[2] + (*it)[0] * image.cols) / (*it)[1]),
                cv::Scalar(255, 255, 255));
    }

    return image;   
}


// void displayWindowEpilines
// (
//   cv::Mat image1, std::vector<cv::KeyPoint> kps1={}, 
//   cv::Mat image2=cv::Mat(), std::vector<cv::KeyPoint> kps2={},
//   std::string name="Epipolar lines", int resizeWidth=1000, int resizeHeight=500, int key=3
// )
// {
//   cv::Mat imageEpiLeft = drawKpts(previousKeyframe_.image.clone(), points2fPrev);
//   imageEpiLeft = drawEpiLines(imageEpiLeft, F, 2, points2fCurr);
//   cv::Mat imageEpiRight = drawKpts(currentFrame_.image.clone(), points2fCurr);
//   drawEpiLines(imageEpiRight, F, 1, points2fPrev);
  
//   displayWindow(imageEpiLeft, imageEpiRight, name, resizeWidth, resizeHeight, key);
// }



void writePointToImg(cv::Mat& image1, cv::Mat& image2, std::vector<cv::KeyPoint> kps1, std::vector<cv::KeyPoint> kps2)
{
  cv::Mat img_kps1; cv::Mat img_kps2;
  if (! kps1.empty())
    cv::drawKeypoints(image1, kps1, img_kps1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  if (! kps2.empty())
    cv::drawKeypoints(image2, kps2, img_kps2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  
  image1 = img_kps1;
  image2 = img_kps2;
}


void saveImgWithKps(cv::Mat image1, cv::Mat image2, std::vector<cv::KeyPoint> kps1, std::vector<cv::KeyPoint> kps2)
{  
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);

  std::string result_path = ros::package::getPath("stereo_frontend") + "/../../results/img_kpts/";
  // Create directory
  mkdir(result_path.c_str(), 0777);


  for(int i = 0; i < kps1.size(); i++) 
  {
    cv::Mat img_kp1 = image1; cv::Mat img_kp2 = image2;
    std::vector<cv::KeyPoint> kp1, kp2;
    kp1.push_back(kps1[i]);
    kp2.push_back(kps2[i]);
    writePointToImg(img_kp1, img_kp2, kp1, kp2);

    std::string padded_i = std::to_string(i);
    while (padded_i.size() < 3)
    {
      padded_i = std::to_string(0) + padded_i;
    }

    cv::imwrite(result_path + std::to_string(i) + "_left.png", img_kp1, compression_params);
    cv::imwrite(result_path + std::to_string(i) + "_right.png", img_kp2, compression_params);
  }

}



void printSummary(ros::Time stamp,
                  double tpf,
                  Eigen::Affine3d T_clcr,
                  int num_landmarks,
                  bool loop_detection)
{
  Eigen::Vector3d euler = T_clcr.linear().eulerAngles(0, 1, 2);
  Eigen::Vector3d euler_abs = euler.cwiseAbs();

  if ( euler_abs.x() > M_PI / 2 )
    euler.x() = euler_abs.x() - M_PI;

  if ( euler_abs.y() > M_PI / 2 )
    euler.y() = euler_abs.y() - M_PI;

  if ( euler_abs.z() > M_PI / 2 )
    euler.z() = euler_abs.z() - M_PI;
  
  euler *= (180.0/M_PI);

  std::cout << '\r' << std::fixed << std::setprecision(2)
            << "\033[1;33mSUMMARY\033[0m \033[3m(C frame)\033[0m: "
            << "FPS=" << std::setw(4) << std::setfill(' ') << 1/tpf << "; "
            << "R=" << "["  << std::setw(6) << std::setfill(' ') << euler.x() 
                    << "," << std::setw(6) << std::setfill(' ') << euler.y() 
                    << "," << std::setw(6) << std::setfill(' ') << euler.z() << "]" << "; "
            << "t=" << "["  << std::setw(5) << std::setfill(' ') << T_clcr.translation().x() 
                    << "," << std::setw(5) << std::setfill(' ') << T_clcr.translation().y() 
                    << "," << std::setw(5) << std::setfill(' ') << T_clcr.translation().z() << "]" << "; "
            << "Landmarks=" << std::setw(3) << std::setfill(' ') << num_landmarks << "; "
            << "Loop="      << loop_detection 
            << std::flush;
}