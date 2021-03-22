#pragma once

#include <Eigen/Geometry> 
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


struct Frame
{
  cv::Mat img_l;
  cv::Mat img_r;
 
  std::vector<cv::KeyPoint> kpts_l; 
  std::vector<cv::KeyPoint> kpts_r; 
  cv::Mat descriptor_l; 
  cv::Mat descriptor_r; 

  Eigen::Affine3d T_r_;

  std::vector<cv::Point3f> world_points;
  pcl::PointCloud<pcl::PointXYZ> cloud;
};


struct Sequencer
{
  Frame current;
  Frame previous;

  void storeImagePair(cv::Mat img_left, cv::Mat img_right);
  void updatePreviousFeatures(std::vector<cv::KeyPoint> kpts_left, std::vector<cv::KeyPoint> kpts_right);
  void storeCloud(pcl::PointCloud<pcl::PointXYZ> cloud);
};


void Sequencer::storeImagePair(cv::Mat img_left, cv::Mat img_right)
{
  // Store left
  if (! current.img_l.empty())
		previous.img_l = current.img_l;
  current.img_l = img_left;

  // Store right
  if (! current.img_r.empty())
		previous.img_r = current.img_r;
  current.img_r = img_right;
}


void Sequencer::updatePreviousFeatures(std::vector<cv::KeyPoint> kpts_left, std::vector<cv::KeyPoint> kpts_right)
{
  previous.kpts_l = kpts_left;
  previous.kpts_r = kpts_right;

  current.kpts_l.clear();
  current.kpts_r.clear();
}


void Sequencer::storeCloud(pcl::PointCloud<pcl::PointXYZ> cloud)
{
  if (! current.cloud.empty())
    previous.cloud = current.cloud;
  
  current.cloud = cloud;
}
