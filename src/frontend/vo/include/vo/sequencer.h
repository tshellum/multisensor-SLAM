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
  std::vector<cv::Mat> keyframes;

  void storeImagePair(cv::Mat img_left, 
                      cv::Mat img_right);

  void storeFeatures(std::vector<cv::KeyPoint> kpts_left, 
                     std::vector<cv::KeyPoint> kpts_right);


  void updatePreviousFeatures(std::vector<cv::KeyPoint> kpts_left, 
                              std::vector<cv::KeyPoint> kpts_right);

  void storeCloud(std::vector<cv::Point3f> world_points,
                  std::vector<int> triangulated_indices);

  void updatePreviousFrame();

  bool isUpToScaleTransformationSmooth(Eigen::Affine3d T_cur, Eigen::Affine3d T_prev, double R_thresh = M_PI/6, double t_thresh = 0.2);
  bool isTransformationSmooth(Eigen::Affine3d T_cur, Eigen::Affine3d T_prev, double R_thresh = M_PI/6, double t_thresh = 1.0);
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


bool Sequencer::isUpToScaleTransformationSmooth(Eigen::Affine3d T_cur, Eigen::Affine3d T_prev, double R_thresh, double t_thresh)
{
  if (T_prev.translation().norm() != 0)
  {
    T_cur.translation() /= T_cur.translation().norm();
    T_prev.translation() /= T_prev.translation().norm();
  }
  
  Eigen::Vector3d rot_euler_cur  = T_cur.linear().eulerAngles(0,1,2).cwiseAbs();
  if ( rot_euler_cur.x() > M_PI/2)
    rot_euler_cur.x() -= M_PI;

  if ( rot_euler_cur.y() > M_PI/2)
    rot_euler_cur.y() -= M_PI;

  if ( rot_euler_cur.z() > M_PI/2)
    rot_euler_cur.z() -= M_PI;

  Eigen::Vector3d rot_euler_prev = T_prev.linear().eulerAngles(0,1,2).cwiseAbs();
    if ( rot_euler_prev.x() > M_PI/2)
    rot_euler_prev.x() -= M_PI;

  if ( rot_euler_prev.y() > M_PI/2)
    rot_euler_prev.y() -= M_PI;

  if ( rot_euler_prev.z() > M_PI/2)
    rot_euler_prev.z() -= M_PI;

  Eigen::Vector3d rot_diff_abs = (rot_euler_cur - rot_euler_prev).cwiseAbs(); 
  
  // bool a = (rot_diff_abs.x() < R_thresh);
  // bool b = (rot_diff_abs.y() < R_thresh);
  // bool c = (rot_diff_abs.z() < R_thresh);
  // bool d = (std::abs(T_cur(2,3)) > std::abs(T_cur(0,3)));
  // bool e = (std::abs(T_cur(2,3)) > std::abs(T_cur(1,3)));
  // bool f = ( (std::abs(T_cur(0,3)) - std::abs(T_prev(0,3)) ) < t_thresh );
  // bool g = ( (std::abs(T_cur(1,3)) - std::abs(T_prev(1,3)) ) < t_thresh );
  // bool h = ( (std::abs(T_cur(2,3)) - std::abs(T_prev(2,3)) ) < t_thresh );
  // bool i = ( std::abs(T_cur(2,3) - T_prev(2,3)) < 1 );

  // if (!a)
  // {
  //   std::cout << "- (rot_diff_abs.x() < R_thresh): " << a << std::endl;
  //   std::cout << "\t- rot_diff_abs.x(): " << rot_diff_abs.x() << ", thresh: " << R_thresh << std::endl;
  // }
  // if (!b)
  // {
  //   std::cout << "- (rot_diff_abs.y() < R_thresh): " << b << std::endl;
  //   std::cout << "\t- rot_diff_abs.y(): " << rot_diff_abs.y() << ", thresh: " << R_thresh << std::endl;
  // }
  // if (!c)
  // {
  //   std::cout << "- (rot_diff_abs.z() < R_thresh): " << c << std::endl;
  //   std::cout << "\t- rot_diff_abs.z(): " << rot_diff_abs.z() << ", thresh: " << R_thresh << std::endl;
  // }
  // if (!d)
  //   std::cout << "- (std::abs(T_cur(2,3)) > std::abs(T_cur(0,3))): " << d << std::endl;
  // if (!e)
  //   std::cout << "- (std::abs(T_cur(2,3)) > std::abs(T_cur(1,3))): " << e << std::endl;
  // if (!f)
  //   std::cout << "- ( (std::abs(T_cur(0,3)) - std::abs(T_prev(0,3)) ) < t_thresh ): " << f << std::endl;
  // if (!g)
  //   std::cout << "- ( (std::abs(T_cur(2,3)) - std::abs(T_prev(1,3)) ) < t_thresh ): " << g << std::endl;
  // if (!h)
  //   std::cout << "- ( (std::abs(T_cur(2,3)) - std::abs(T_prev(2,3)) ) < t_thresh ): " << h << std::endl;
  // if (!i)
  //   std::cout << "- ( std::abs(T_cur(2,3) - T_prev(2,3)) < 1 ): " << i << std::endl;

  // No major difference in rotational estimates since prev
  return ( (rot_diff_abs.x() < R_thresh)  
        && (rot_diff_abs.y() < R_thresh)  
        && (rot_diff_abs.z() < R_thresh) 
        && (std::abs(T_cur(2,3)) > std::abs(T_cur(0,3)))        // No dominant sideways motion  
        && (std::abs(T_cur(2,3)) > std::abs(T_cur(1,3)))        // No dominant upwards/downwards motion  
        && ( (std::abs(T_cur(0,3)) - std::abs(T_prev(0,3)) ) < t_thresh )        
        && ( (std::abs(T_cur(1,3)) - std::abs(T_prev(1,3)) ) < t_thresh )        
        && ( (std::abs(T_cur(2,3)) - std::abs(T_prev(2,3)) ) < t_thresh )        
        && ( std::abs(T_cur(2,3) - T_prev(2,3)) < 1 )        
  );
}


bool Sequencer::isTransformationSmooth(Eigen::Affine3d T_cur, Eigen::Affine3d T_prev, double R_thresh, double t_thresh)
{
  Eigen::Vector3d rot_euler_cur  = T_cur.linear().eulerAngles(0,1,2).cwiseAbs();
  if ( rot_euler_cur.x() > M_PI/2)
    rot_euler_cur.x() -= M_PI;

  if ( rot_euler_cur.y() > M_PI/2)
    rot_euler_cur.y() -= M_PI;

  if ( rot_euler_cur.z() > M_PI/2)
    rot_euler_cur.z() -= M_PI;

  Eigen::Vector3d rot_euler_prev = T_prev.linear().eulerAngles(0,1,2).cwiseAbs();
    if ( rot_euler_prev.x() > M_PI/2)
    rot_euler_prev.x() -= M_PI;

  if ( rot_euler_prev.y() > M_PI/2)
    rot_euler_prev.y() -= M_PI;

  if ( rot_euler_prev.z() > M_PI/2)
    rot_euler_prev.z() -= M_PI;

  Eigen::Vector3d rot_diff_abs = (rot_euler_cur - rot_euler_prev).cwiseAbs(); 


  // bool a = (rot_diff_abs.x() < R_thresh);
  // bool b = (rot_diff_abs.y() < R_thresh);
  // bool c = (rot_diff_abs.z() < R_thresh);
  // bool d = (std::abs(T_cur(2,3)) > std::abs(T_cur(0,3)));
  // bool e = (std::abs(T_cur(2,3)) > std::abs(T_cur(1,3)));
  // bool f = ( (std::abs(T_cur(0,3)) - std::abs(T_prev(0,3)) ) < t_thresh );
  // bool g = ( (std::abs(T_cur(1,3)) - std::abs(T_prev(1,3)) ) < t_thresh );
  // bool h = ( (std::abs(T_cur(2,3)) - std::abs(T_prev(2,3)) ) < t_thresh );
  // bool j = (T_cur(2,3) >= -0.3);
  // bool k = ( T_cur.translation().norm() < 2 );

  // if (!a)
  // {
  //   std::cout << "- (rot_diff_abs.x() < R_thresh): " << a << std::endl;
  //   std::cout << "\t- rot_diff_abs.x(): " << rot_diff_abs.x() << ", thresh: " << R_thresh << std::endl;
  // }
  // if (!b)
  // {
  //   std::cout << "- (rot_diff_abs.y() < R_thresh): " << b << std::endl;
  //   std::cout << "\t- rot_diff_abs.y(): " << rot_diff_abs.y() << ", thresh: " << R_thresh << std::endl;
  // }
  // if (!c)
  // {
  //   std::cout << "- (rot_diff_abs.z() < R_thresh): " << c << std::endl;
  //   std::cout << "\t- rot_diff_abs.z(): " << rot_diff_abs.z() << ", thresh: " << R_thresh << std::endl;
  // }
  // if (!d)
  //   std::cout << "- (std::abs(T_cur(2,3)) > std::abs(T_cur(0,3))): " << d << std::endl;
  // if (!e)
  //   std::cout << "- (std::abs(T_cur(2,3)) > std::abs(T_cur(1,3))): " << e << std::endl;
  // if (!f)
  //   std::cout << "- ( (std::abs(T_cur(0,3)) - std::abs(T_prev(0,3)) ) < t_thresh ): " << f << std::endl;
  // if (!g)
  //   std::cout << "- ( (std::abs(T_cur(2,3)) - std::abs(T_prev(1,3)) ) < t_thresh ): " << g << std::endl;
  // if (!h)
  // {
  //   std::cout << "- ( (std::abs(T_cur(2,3)) - std::abs(T_prev(2,3)) ) < t_thresh ): " << h << std::endl;
  //   std::cout << "\t- std::abs(T_cur(2,3)): " << std::abs(T_cur(2,3)) << ", std::abs(T_prev(2,3)): " << std::abs(T_prev(2,3)) << std::endl;
  // }
  // if (!j)
  //   std::cout << "- (T_cur(2,3) >= -0.3): " << j << std::endl;
  // if (!k)
  //   std::cout << "- (T_cur.translation().norm() < 2): " << k << std::endl;


  // No major difference in rotational estimates since prev
  return ( (rot_diff_abs.x() < R_thresh)  
        && (rot_diff_abs.y() < R_thresh)  
        && (rot_diff_abs.z() < R_thresh) 
        && (T_cur(2,3) >= -0.3)
        && (std::abs(T_cur(2,3)) >= std::abs(T_cur(0,3)))        // No dominant sideways motion: z < x (in camera frame)
        && (std::abs(T_cur(2,3)) >= std::abs(T_cur(1,3)))        // No dominant upwards/downwards motion: z < y (in camera frame)
        && ( (std::abs(T_cur(0,3)) - std::abs(T_prev(0,3)) ) < t_thresh )        
        && ( (std::abs(T_cur(1,3)) - std::abs(T_prev(1,3)) ) < t_thresh )        
        // && ( (std::abs(T_cur(2,3)) - std::abs(T_prev(2,3)) ) < t_thresh )        
        && (T_cur.translation().norm() < 3)     
        // && ( T_cur.translation().norm() > 0.05 )     
  );
}
