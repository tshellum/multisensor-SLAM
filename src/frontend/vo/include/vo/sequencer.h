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

  Eigen::Affine3d T_r;
  double scale;

  std::vector<cv::Point3f> world_points;
  std::vector<int> indices;

  Frame()
  : T_r(Eigen::Affine3d::Identity())
  , scale(1.0)
  {}
};


struct Sequencer
{
  Frame current;
  Frame previous;

  void storeImagePair(cv::Mat img_left, 
                      cv::Mat img_right);

  void storeFeatures(std::vector<cv::KeyPoint> kpts_left, 
                     std::vector<cv::KeyPoint> kpts_right);


  void updatePreviousFeatures(std::vector<cv::KeyPoint> kpts_left, 
                              std::vector<cv::KeyPoint> kpts_right);

  void storeCloud(std::vector<cv::Point3f> world_points,
                  std::vector<int> triangulated_indices);

  void updatePreviousFrame();
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


void Sequencer::storeFeatures(std::vector<cv::KeyPoint> kpts_left, std::vector<cv::KeyPoint> kpts_right)
{
  if (! current.kpts_l.empty() && ! current.kpts_r.empty())
  {
    previous.kpts_l = current.kpts_l;
    previous.kpts_r = current.kpts_r;
  }
  
  current.kpts_l = kpts_left;
  current.kpts_r = kpts_right;
}

void Sequencer::updatePreviousFeatures(std::vector<cv::KeyPoint> kpts_left, std::vector<cv::KeyPoint> kpts_right)
{
  previous.kpts_l = kpts_left;
  previous.kpts_r = kpts_right;

  current.kpts_l.clear();
  current.kpts_r.clear();
}



void Sequencer::storeCloud(std::vector<cv::Point3f> world_points,
                           std::vector<int> triangulated_indices)
{
  if (! current.world_points.empty())
  {
    previous.world_points = current.world_points;
    previous.indices = current.indices;
  }
  
  current.world_points = world_points;
  current.indices = triangulated_indices;
}


void Sequencer::updatePreviousFrame()
{
  previous = current;
  
  current.img_l.release();
  current.img_r.release();
  current.kpts_l.clear();
  current.kpts_r.clear();
  current.descriptor_l.release();
  current.descriptor_r.release();
  current.T_r = Eigen::Affine3d::Identity();
  current.world_points.clear();
  current.indices.clear();
}