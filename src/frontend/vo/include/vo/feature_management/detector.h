#pragma once

#include <algorithm> 
#include <array>
#include <iterator> // required for std::size

#include <boost/property_tree/ptree.hpp>

#include <opencv2/video.hpp>

#ifdef OPENCV_CUDA_ENABLED
  #include <opencv2/cudafeatures2d.hpp>
#else
  #include "opencv2/features2d.hpp"
#endif


class Detector
{
private:
  // Detector
  cv::Ptr<cv::Feature2D> extractor_;
  cv::Ptr<cv::Feature2D> descriptor_; 

  // Function parameters
  int grid_size_;
  int width_, height_;
  int patch_w_, patch_h_;
  int n_buckets_;
  int max_features_;
  int nms_distance_;

  // Point parameters    
  int n_id_;

public:
  Detector(){}

  Detector(ros::NodeHandle nh) 
  : n_id_(0)
  {
    // Detector parameters
    int threshold;
    nh.getParam("/detector/feature_threshold", threshold);
    nh.getParam("/detector/max_number_of_features", max_features_);
    nh.getParam("/detector/grid_size", grid_size_);
    nh.getParam("/detector/nms_distance", nms_distance_);
    
    std::string extractor_type, descriptor_type;
    nh.getParam("/detector/extractor_type", extractor_type);
    nh.getParam("/detector/descriptor_type", descriptor_type);

    int img_width_full, img_height_full;
    nh.getParam("/camera_left/image_width", img_width_full);
    nh.getParam("/camera_left/image_height", img_height_full);

    // Extractor
    if (extractor_type == "GFFT")
      extractor_ = cv::GFTTDetector::create(max_features_, // maximum number of features
                                            0.01,         // quality level
                                            10);          // minimum allowed distance
    else if (extractor_type == "FAST")
      extractor_ = cv::FastFeatureDetector::create(threshold, // threshold
                                                     true);      // NMS
    else if (extractor_type == "ORB")
      extractor_ = cv::ORB::create();
    else
      extractor_ = cv::FastFeatureDetector::create();
                      

    // Descriptor                          
    if (descriptor_type == "ORB")
      descriptor_ = cv::ORB::create(max_features_);
    else 
      descriptor_ = cv::ORB::create();
  

    width_ = img_width_full - (img_width_full % grid_size_);
    height_ = img_height_full - (img_height_full % grid_size_);

    patch_w_ = width_ / grid_size_;
    patch_h_ = height_ / grid_size_;

    n_buckets_ = grid_size_*grid_size_;
  }

  Detector(boost::property_tree::ptree detector_config,
           boost::property_tree::ptree camera_config)
  : n_id_(0)
  {
    // Detector parameters
    int threshold = detector_config.get< int >("detector.feature_threshold");
    max_features_ = detector_config.get< int >("detector.max_number_of_features");
    grid_size_    = detector_config.get< int >("detector.grid_size");
    nms_distance_ = detector_config.get< int >("detector.nms_distance");

    std::string extractor_type  = detector_config.get< std::string >("detector.extractor_type");
    std::string descriptor_type = detector_config.get< std::string >("detector.descriptor_type");

    int img_width_full  = camera_config.get< int >("image_width");
    int img_height_full = camera_config.get< int >("image_height");	

    // Construct detector
    #ifdef OPENCV_CUDA_ENABLED
      extractor_ = cv::cuda::FastFeatureDetector::create(threshold, // threshold
                                                   true);     // NMS
      descriptor_ = cv::cuda::ORB::create(max_features_);
    #else
      extractor_ = cv::FastFeatureDetector::create(threshold, // threshold
                                                   true);     // NMS
      descriptor_ = cv::ORB::create(max_features_);
    #endif
  

    width_ = img_width_full - (img_width_full % grid_size_);
    height_ = img_height_full - (img_height_full % grid_size_);

    patch_w_ = width_ / grid_size_;
    patch_h_ = height_ / grid_size_;

    n_buckets_ = grid_size_*grid_size_;
  };

  ~Detector() {}


  int getDefaultNorm() {return descriptor_->defaultNorm();};

  void bucketedFeatureDetection(cv::Mat image, 
                                std::vector<cv::KeyPoint>& features);
  
  cv::Mat computeDescriptor(cv::Mat image,
                            std::vector<cv::KeyPoint>& features);

  std::pair< std::vector<cv::KeyPoint>, cv::Mat > detectCompute(cv::Mat image);
};


// TODO: Dynamisk threshold
void Detector::bucketedFeatureDetection(cv::Mat image, std::vector<cv::KeyPoint>& features)
{
  // Place tracked features from previous frame in buckets
  std::vector<cv::KeyPoint> feature_buckets[n_buckets_];
  for (cv::KeyPoint& kpt : features)
    feature_buckets[int(kpt.pt.x / patch_w_) * grid_size_ + int(kpt.pt.y / patch_h_)].push_back(kpt);
  
  
  // Detect new features w/ NMS
  std::vector<cv::KeyPoint> patch_features; 
  int it_bucket = 0;

  for (unsigned int x = 0; x < width_; x += patch_w_)
  {
    for (unsigned int y = 0; y < height_; y += patch_h_)
    {
      // Detect features
      cv::Mat mask = cv::Mat();
      cv::Mat img_patch_cur = image(cv::Rect(x, y, patch_w_, patch_h_)).clone();
      
      extractor_->detect(img_patch_cur, patch_features, mask); 

      // Sort to retain strongest features
      std::sort(patch_features.begin(), patch_features.end(), 
          [](const cv::KeyPoint& kpi, const cv::KeyPoint& kpj){ return kpi.response > kpj.response;  }
      ); 

      // Retain tracked features and only add strongest features
      int num_bucket_features = 0;
      for (cv::KeyPoint& new_feature : patch_features)
      { 
        // Not more that max_features number of features in each bucket 
        if ( (feature_buckets[it_bucket].size() + num_bucket_features) > max_features_ )
          break;

        // Correct for patch position
        new_feature.pt.x += x;
        new_feature.pt.y += y; 

        int x0 = new_feature.pt.x;
        int y0 = new_feature.pt.y;

        bool skip = false;
        for (cv::KeyPoint& existing_feature : feature_buckets[it_bucket]) 
        {
          // NMS: Skip point if point is in vicinity of existing points
          if ( pow((existing_feature.pt.x - x0), 2) + pow((existing_feature.pt.y - y0), 2) < pow(nms_distance_, 2) )
          {
            skip = true;
            break;
          }                        
        }

        if (skip)
            continue;

        new_feature.class_id = n_id_++; 

        features.push_back(new_feature);
        num_bucket_features++;
      }

      it_bucket++;
    }
  }
}


cv::Mat Detector::computeDescriptor(cv::Mat image,
                                    std::vector<cv::KeyPoint>& features)
{
  cv::Mat descriptor;
  descriptor_->compute(image, features, descriptor); 
  return descriptor;
}


std::pair< std::vector<cv::KeyPoint>, cv::Mat > Detector::detectCompute(cv::Mat image)
{
  std::vector<cv::KeyPoint> kpts;
  cv::Mat desc;
  
  descriptor_->detectAndCompute(image, cv::Mat(), kpts, desc); 

  return std::make_pair(kpts, desc);
}