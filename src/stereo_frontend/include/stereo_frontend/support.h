#pragma once

#include <ros/package.h>

#include <boost/algorithm/string.hpp>

#include <opencv2/opencv.hpp>

#include <sys/stat.h> 


void NotImplementedError(std::string function_name, std::string filename) // call by NotImplementedError(__func__, __FILE__);
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


void displayWindowFeatures
(
  cv::Mat image1, std::vector<cv::KeyPoint> kps1={}, 
  cv::Mat image2=cv::Mat(), std::vector<cv::KeyPoint> kps2={},
  std::string name="Detections", int resizeWidth=1000, int resizeHeight=500, int key=3
)
{
  cv::Mat img_kps1 = image1; cv::Mat img_kps2;
  if (! kps1.empty())
    cv::drawKeypoints(image1, kps1, img_kps1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  if (! kps2.empty())
    cv::drawKeypoints(image2, kps2, img_kps2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

  displayWindow(img_kps1, img_kps2, name, resizeWidth, resizeHeight, key);
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

