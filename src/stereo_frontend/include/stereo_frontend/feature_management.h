#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include "opencv2/features2d.hpp"

#include "stereo_frontend/support.h"


struct Frame
{
    cv::Mat image; 
    cv::Mat descriptor; 
    std::vector<cv::KeyPoint> features; 
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
                                MAX_FEATURES_, // maximum number of features
                                0.01,          // quality level
                                10);           // minimum allowed distance

        descriptor_ = cv::ORB::create(MAX_FEATURES_);
    }
    ~FeatureManager() {};

    void initiate_frames(cv::Mat image_left, cv::Mat image_right);

    cv::Mat get_prev_image_left()  {return left_prev_.image;};
    cv::Mat get_prev_image_right() {return right_prev_.image;};
    cv::Mat get_cur_image_left()   {return left_cur_.image;};
    cv::Mat get_cur_image_right()  {return right_cur_.image;};

    std::vector<cv::KeyPoint> get_prev_features_left()  {return left_prev_.features;};
    std::vector<cv::KeyPoint> get_prev_features_right() {return right_prev_.features;};
    std::vector<cv::KeyPoint> get_cur_features_left()   {return left_cur_.features;};
    std::vector<cv::KeyPoint> get_cur_features_right()  {return right_cur_.features;};

    int get_num_features_left_prev()  {return left_prev_.features.size();};
    int get_num_features_right_prev() {return right_prev_.features.size();};
    int get_num_features_left_cur()   {return left_cur_.features.size();};
    int get_num_features_right_cur()  {return right_cur_.features.size();};

    int get_default_norm();

    void set_cur_frames();

    void printit() {ROS_INFO("print");};
    void detectAndCompute();
    void bucketedFeatureDetection();
    void track(cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::KeyPoint>& prev_kps, std::vector<cv::KeyPoint>& cur_kps);
    void track_stereo_features();
    
};



void FeatureManager::initiate_frames(cv::Mat image_left, cv::Mat image_right)
{
    left_cur_.image = image_left; 
    left_cur_.descriptor = cv::Mat(); 
    left_cur_.features.clear(); 

    right_cur_.image = image_right; 
    right_cur_.descriptor = cv::Mat(); 
    right_cur_.features.clear(); 
}

void FeatureManager::set_cur_frames()
{
    left_prev_ = left_cur_;
    right_prev_ = right_cur_;
}

int FeatureManager::get_default_norm()
{
    return descriptor_->defaultNorm();
}


void FeatureManager::detectAndCompute()  
{        
    extractor_->detect(left_cur_.image, left_cur_.features, cv::Mat()); 
    extractor_->detect(right_cur_.image, right_cur_.features, cv::Mat()); 

    descriptor_->compute(left_cur_.image, left_cur_.features, left_cur_.descriptor); 
    descriptor_->compute(right_cur_.image, right_cur_.features, right_cur_.descriptor); 
}    



void FeatureManager::bucketedFeatureDetection()
{
    NotImplementedError(__func__, __FILE__); 
}


void FeatureManager::track(cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::KeyPoint>& prev_kps, std::vector<cv::KeyPoint>& cur_kps)
{
    // Point vectors
    std::size_t n_pts = prev_kps.size(); 
    std::vector<cv::Point2f> prev_pts(n_pts), cur_pts(n_pts);

    for (std::size_t i = 0; i < n_pts; ++i)
        prev_pts[i] = prev_kps[i].pt; 

    std::vector<uchar> status; 
    std::vector<float> error; 

    // https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
    cv::calcOpticalFlowPyrLK(
        prev_img, 
        cur_img, 
        prev_pts, 
        cur_pts, 
        status, 
        error
    );


    std::vector<cv::KeyPoint> tracked_kpts;
    for(uint i = 0; i < n_pts; ++i)
    {
        // Select good points
        if (
            (status[i] == 1) 
            && (cur_pts[i].x > 0) && (cur_pts[i].y > 0)
            && (cur_pts[i].x < prev_img.cols) && (cur_pts[i].y < prev_img.rows)
        ) 
        {
            tracked_kpts.push_back(
                cv::KeyPoint
                (
                    cur_pts[i], 
                    prev_kps[i].size, 
                    prev_kps[i].angle, 
                    prev_kps[i].response, 
                    prev_kps[i].octave, 
                    prev_kps[i].class_id
                )
            );
        }
    }

    cur_kps = tracked_kpts;
}

void FeatureManager::track_stereo_features()
{
    printit();
    track(left_prev_.image, left_cur_.image, left_prev_.features, left_cur_.features);
    track(right_prev_.image, right_cur_.image, right_prev_.features, right_cur_.features);
}