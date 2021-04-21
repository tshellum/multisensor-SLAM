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


// enum ExtractorType 
// {
  
// };


class Detector
{
private:
  // Detector
  cv::Ptr<cv::Feature2D> extractor_;
  cv::Ptr<cv::Feature2D> descriptor_; 

  bool use_bucketed_procedure_;
  bool use_nms_;
  bool use_provided_features_;

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

  Detector(boost::property_tree::ptree detector_config,
           int image_width, 
           int image_height)
  : n_id_(0)
  {
    use_bucketed_procedure_ = detector_config.get< bool >("detector.use_buckets");
    use_nms_ = detector_config.get< bool >("detector.use_nms");
    use_provided_features_ = detector_config.get< bool >("detector.use_provided_features");

    // Detector parameters
    max_features_ = detector_config.get< int >("detector.patch_feature_limit");
    grid_size_    = detector_config.get< int >("detector.grid_size");
    nms_distance_ = detector_config.get< int >("detector.nms_distance");

    std::string extractor_type  = detector_config.get< std::string >("detector.extractor_type");
    std::string descriptor_type = detector_config.get< std::string >("detector.descriptor_type");


    // Construct detector
    #ifdef OPENCV_CUDA_ENABLED
      if (extractor_type == "FAST")
        extractor_ = cv::cuda::FastFeatureDetector::create( detector_config.get< int >("detector.FAST.threshold"),
                                                            detector_config.get< bool >("detector.FAST.nonmax_suppression") );  
      else if (extractor_type == "ORB")
        extractor_ = cv::cuda::ORB::create( detector_config.get< int >("detector.ORB.nfeatures"),
                                            detector_config.get< float >("detector.ORB.scale_factor"),
                                            detector_config.get< int >("detector.ORB.nlevels"),
                                            detector_config.get< int >("detector.ORB.edge_threshold"),
                                            detector_config.get< int >("detector.ORB.first_level"),
                                            detector_config.get< int >("detector.ORB.WTA_K"),
                                            cv::ORB::ScoreType( detector_config.get< int >("detector.ORB.score_type") ),
                                            detector_config.get< int >("detector.ORB.patch_size"),
                                            detector_config.get< int >("detector.ORB.fast_threshold") );
      
      else if (extractor_type == "GFFT")
        extractor_ = cv::cuda::GFTTDetector::create( detector_config.get< int >("detector.GFFT.max_corners"),
                                                     detector_config.get< float >("detector.GFFT.quality_level"),
                                                     detector_config.get< int >("detector.GFFT.min_distance"),
                                                     detector_config.get< int >("detector.GFFT.block_size"),
                                                     detector_config.get< bool >("detector.GFFT.use_harris_detector"),
                                                     detector_config.get< float >("detector.GFFT.k") );         
      else if (extractor_type == "AKAZE")
        extractor_ = cv::cuda::AKAZE::create( cv::AKAZE::DescriptorType( detector_config.get< int >("detector.AKAZE.descriptor_type") ),
                                              detector_config.get< int >("detector.AKAZE.descriptor_size"),
                                              detector_config.get< int >("detector.AKAZE.descriptor_channels"),
                                              detector_config.get< int >("detector.AKAZE.threshold"),
                                              detector_config.get< int >("detector.AKAZE.nOctaves"),
                                              detector_config.get< int >("detector.AKAZE.nOctave_layers"),
                                              cv::KAZE::DiffusivityType( detector_config.get< int >("detector.AKAZE.diffusivity") ) );

      else
        extractor_ = cv::cuda::FastFeatureDetector::create();

      descriptor_ = cv::cuda::ORB::create(max_features_);

    #else
      if (extractor_type == "FAST")
        extractor_ = cv::FastFeatureDetector::create( detector_config.get< int >("detector.FAST.threshold"),
                                                      detector_config.get< bool >("detector.FAST.nonmax_suppression") );  
      else if (extractor_type == "ORB")
        extractor_ = cv::ORB::create( detector_config.get< int >("detector.ORB.nfeatures"),
                                      detector_config.get< float >("detector.ORB.scale_factor"),
                                      detector_config.get< int >("detector.ORB.nlevels"),
                                      detector_config.get< int >("detector.ORB.edge_threshold"),
                                      detector_config.get< int >("detector.ORB.first_level"),
                                      detector_config.get< int >("detector.ORB.WTA_K"),
                                      cv::ORB::ScoreType( detector_config.get< int >("detector.ORB.score_type") ),
                                      detector_config.get< int >("detector.ORB.patch_size"),
                                      detector_config.get< int >("detector.ORB.fast_threshold") );
      
      else if (extractor_type == "GFFT")
        extractor_ = cv::GFTTDetector::create( detector_config.get< int >("detector.GFFT.max_corners"),
                                               detector_config.get< float >("detector.GFFT.quality_level"),
                                               detector_config.get< int >("detector.GFFT.min_distance"),
                                               detector_config.get< int >("detector.GFFT.block_size"),
                                               detector_config.get< bool >("detector.GFFT.use_harris_detector"),
                                               detector_config.get< float >("detector.GFFT.k") );         
      else if (extractor_type == "AKAZE")
        extractor_ = cv::AKAZE::create( cv::AKAZE::DescriptorType( detector_config.get< int >("detector.AKAZE.descriptor_type") ),
                                        detector_config.get< int >("detector.AKAZE.descriptor_size"),
                                        detector_config.get< int >("detector.AKAZE.descriptor_channels"),
                                        detector_config.get< int >("detector.AKAZE.threshold"),
                                        detector_config.get< int >("detector.AKAZE.nOctaves"),
                                        detector_config.get< int >("detector.AKAZE.nOctave_layers"),
                                        cv::KAZE::DiffusivityType( detector_config.get< int >("detector.AKAZE.diffusivity") ) );

      else
        extractor_ = cv::FastFeatureDetector::create();

      descriptor_ = cv::ORB::create();
    #endif
  

    width_ = image_width - (image_width % grid_size_);
    height_ = image_height - (image_height % grid_size_);

    patch_w_ = width_ / grid_size_;
    patch_h_ = height_ / grid_size_;

    n_buckets_ = grid_size_*grid_size_;
  };

  ~Detector() {}


  int getDefaultNorm() {return descriptor_->defaultNorm();};

  void detect(cv::Mat image, 
              std::vector<cv::KeyPoint>& features);

  void bucketedFeatureDetection(cv::Mat image, 
                                std::vector<cv::KeyPoint>& features);
  
  cv::Mat computeDescriptor(cv::Mat image,
                            std::vector<cv::KeyPoint>& features);

  std::pair< std::vector<cv::KeyPoint>, cv::Mat > detectCompute(cv::Mat image);
};



void Detector::detect(cv::Mat image, 
                      std::vector<cv::KeyPoint>& features)
{
  if (use_bucketed_procedure_)
    bucketedFeatureDetection(image, features);

  else
  { 
    std::vector<cv::KeyPoint> features_new;
    extractor_->detect(image, features_new, cv::Mat());  

    std::vector<cv::KeyPoint> features_concatenated;
    if (use_provided_features_)
      features_concatenated = features;

    for(unsigned int i = 0; i < features_new.size(); i++)
    {
      bool add_pt = true;
      if (use_nms_)
      {
        for(unsigned int j = 0; j < features_concatenated.size(); j++)
        {
          // NMS
          if ( pow((features_new[i].pt.x - features_concatenated[j].pt.x), 2) 
            + pow((features_new[i].pt.y - features_concatenated[j].pt.y), 2) < pow(nms_distance_, 2) )
          {
            add_pt = false;
            break;
          }
        }
      }

      features_new[i].class_id = n_id_++;
      if (add_pt)
        features_concatenated.push_back(features_new[i]);
    }
    features = features_concatenated;
  }
}


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