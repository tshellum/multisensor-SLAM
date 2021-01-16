#pragma once

#include "opencv2/features2d.hpp"

struct Frame
{
    cv::Mat image; 
    cv::Mat descriptor; 
    std::vector<cv::KeyPoint> keypoints; 
};


struct Point 
{
    // Point
    int id;
};



class FeatureManager
{
private:
    // Frames
    Frame left_prev_;
    Frame right_prev_;
    Frame left_cur_;
    Frame right_cur_;

    // Detectors
    cv::Ptr<cv::GFTTDetector> extractor_; 
    cv::Ptr<cv::Feature2D>    descriptor_; 

    // Detector parameters
    unsigned int MAX_FEATURES_;
    unsigned int GRID_SIZE_; // NxN size grid
    int THRESHOLD_;

    // Point parameters    
    int n_id_;


public:
    FeatureManager(int MAX_FEATURES = 1000, int GRID_SIZE = 20, int THRESHOLD = 25) 
    : GRID_SIZE_(GRID_SIZE), MAX_FEATURES_(MAX_FEATURES), THRESHOLD_(THRESHOLD),
      n_id_(0)
    {
        extractor_ = cv::GFTTDetector::create(
                                MAX_FEATURES_, // maximum number of keypoints
                                0.01,          // quality level
                                10);           // minimum allowed distance

        descriptor_ = cv::ORB::create(MAX_FEATURES_);
    }
    ~FeatureManager() {};

    void initiate_frames(cv::Mat image_left, cv::Mat image_right);

    cv::Mat get_image_left()  {return left_.image;};
    cv::Mat get_image_right() {return right_.image;};

    std::vector<cv::KeyPoint> get_keypoints_left()  {return left_.keypoints;};
    std::vector<cv::KeyPoint> get_keypoints_right() {return right_.keypoints;};

    int get_default_norm();
    void detectAndCompute();
    void gridDetectAndCompute();
};


void FeatureManager::initiate_frames(cv::Mat image_left, cv::Mat image_right)
{
    left_.image = image_left; 
    left_.descriptor = cv::Mat(); 
    left_.keypoints.clear(); 

    right_.image = image_right; 
    right_.descriptor = cv::Mat(); 
    right_.keypoints.clear(); 
}

int FeatureManager::get_default_norm()
{
    return descriptor_->defaultNorm();
}


void FeatureManager::detectAndCompute()  
{        
    extractor_->detect(left_.image, left_.keypoints, cv::Mat()); 
    extractor_->detect(right_.image, right_.keypoints, cv::Mat()); 

    descriptor_->compute(left_.image, left_.keypoints, left_.descriptor); 
    descriptor_->compute(right_.image, right_.keypoints, right_.descriptor); 
}    



// void FeatureManager::gridDetectAndCompute()
// {
//     std::vector<cv::KeyPoint> subKeypoints; 

//     unsigned int nCols = frame.image.cols; 
//     unsigned int nRows = frame.image.rows; 

//     unsigned int dCols = frame.image.cols / GRID_SIZE_; 
//     unsigned int dRows = frame.image.rows / GRID_SIZE_; 

//     if (!(frame.image.cols % GRID_SIZE_) && !(frame.image.rows % GRID_SIZE_) || false)
//     {
//         for (unsigned int x = 0; x < nCols; x += dCols)
//         {
//             for (unsigned int y = 0; y < nRows; y += dRows)
//             {
//                 cv::Mat mask = cv::Mat();
//                 cv::Mat subimg = frame.image(cv::Rect(x, y, dCols, dRows)).clone();
//                 extractor_->detect(subimg, subKeypoints, mask); 

//                 for (cv::KeyPoint& kp : subKeypoints)
//                 {
//                     kp.pt.x += x;
//                     kp.pt.y += y; 
                    
//                     frame.keypoints.push_back(kp);
//                 }
//             }
//         }
//     }
//     else
//     {
//         // ROS_INFO("NOT VALID DIMENTIONS");
//         extractor_->detect(frame.image, frame.keypoints, cv::Mat()); 
//     }
    

//     // Non-Max supression 
//     std::sort(frame.keypoints.begin(), frame.keypoints.end(), 
//         [](const cv::KeyPoint& kpi, const cv::KeyPoint& kpj){ return kpi.response > kpj.response;  }
//     ); 
//     frame.keypoints = ssc(frame.keypoints, NUM_RETURN_POINTS_, TOLERANCE_, frame.image.cols, frame.image.rows);        
    

//     descriptor_->compute(frame.image, frame.keypoints, frame.descriptor); 
// }