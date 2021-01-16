#pragma once

#include <boost/algorithm/string.hpp>


void NotImplementedError(std::string function_name, std::string filename)
{
  std::vector<std::string> entries;
  boost::split(entries, filename, boost::is_any_of("/"));

  ROS_ERROR_STREAM("Function " << function_name << "() in " << entries.rbegin()[1] << "/" << entries.rbegin()[0] << " not implemented.");
}


void displayWindow
(
  cv::Mat image1, cv::Mat image2=cv::Mat(), 
  std::string name="Stereo images", int resizeWidth=1000, int resizeHeight=500, int key=3
)
{
  if (image2.empty())
  {
    cv::namedWindow(name, cv::WINDOW_NORMAL);
    cv::imshow(name, image1);
    cv::resizeWindow(name, resizeWidth, resizeHeight);
    cv::waitKey(key);
  }
  else
  {
    cv::Mat splitimage;
    cv::hconcat(image1, image2, splitimage);
    
    cv::namedWindow(name, cv::WINDOW_NORMAL);
    cv::imshow(name, splitimage);
    cv::resizeWindow(name, resizeWidth, resizeHeight);
    cv::waitKey(key);
  }
}


void displayWindowKeypoints
(
  cv::Mat image1, cv::Mat image2=cv::Mat(), 
  std::vector<cv::KeyPoint> kps1={}, std::vector<cv::KeyPoint> kps2={},
  std::string name="Detections", int resizeWidth=1000, int resizeHeight=500, int key=3
)
{
  cv::Mat img_kps1, img_kps2;
  cv::drawKeypoints(image1, kps1, img_kps1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  if (! image2.empty())
    cv::drawKeypoints(image2, kps2, img_kps2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  
  displayWindow(img_kps1, img_kps2, name, resizeWidth, resizeHeight, key);
}

