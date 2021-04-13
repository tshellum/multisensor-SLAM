#pragma once

#include <ros/package.h>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <opencv2/opencv.hpp>

#include <Eigen/Geometry> 


#include <sys/stat.h> 


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


void printSummary(Eigen::Affine3d T_clcr,
                  int num_landmarks,
                  bool loop_detection)
{
  Eigen::Vector3d euler = T_clcr.linear().eulerAngles(0, 1, 2);
  ROS_INFO_STREAM("Keyframe summary (in image frame) : "
    << "\n\t - Number of landmarks : " << num_landmarks
    << "\n\t - Rotation : " << "[" << euler.x() << ", " << euler.y() << ", " << euler.z() << "]"
    << "\n\t - Translation : " << "[" << T_clcr.translation().x() << ", " << T_clcr.translation().y() << ", " << T_clcr.translation().z() << "]"
    << "\n\t - Loop detection? : " << loop_detection
    << "\n");
}