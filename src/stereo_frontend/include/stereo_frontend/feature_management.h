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
    Frame _left_prev;
    Frame _right_prev;
    Frame _left_cur;
    Frame _right_cur;

    // Detectors
    cv::Ptr<cv::GFTTDetector> _extractor; 
    cv::Ptr<cv::Feature2D>    _descriptor; 

    // Detector parameters
    unsigned int _MAX_FEATURES;
    unsigned int _GRID_SIZE; // NxN size grid
    int _THRESHOLD;

    // Point parameters    
    int _n_id;


public:
    FeatureManager(int MAX_FEATURES = 1000, int GRID_SIZE = 20, int THRESHOLD = 25) 
    : _GRID_SIZE(GRID_SIZE), _MAX_FEATURES(MAX_FEATURES), _THRESHOLD(THRESHOLD),
      _n_id(0)
    {
        _extractor = cv::GFTTDetector::create(
                                _MAX_FEATURES, // maximum number of features
                                0.01,          // quality level
                                10);           // minimum allowed distance

        _descriptor = cv::ORB::create(_MAX_FEATURES);
    }
    ~FeatureManager() {};

    void initiateFrames(cv::Mat image_left, cv::Mat image_right);

    cv::Mat getPrevImageLeft()  {return _left_prev.image;};
    cv::Mat getPrevImageRight() {return _right_prev.image;};
    cv::Mat getCurImageLeft()   {return _left_cur.image;};
    cv::Mat getCurImageRight()  {return _right_cur.image;};

    std::vector<cv::KeyPoint> getPrevFeaturesLeft()  {return _left_prev.features;};
    std::vector<cv::KeyPoint> getPrevFeaturesRight() {return _right_prev.features;};
    std::vector<cv::KeyPoint> getCurFeaturesLeft()   {return _left_cur.features;};
    std::vector<cv::KeyPoint> getCurFeaturesRight()  {return _right_cur.features;};

    int getNumFeaturesLeftPrev()  {return _left_prev.features.size();};
    int getNumFeaturesRightPrev() {return _right_prev.features.size();};
    int getNumFeaturesLeftCur()   {return _left_cur.features.size();};
    int getNumFeaturesRightCur()  {return _right_cur.features.size();};

    int getDefaultNorm() {return _descriptor->defaultNorm();};

    void updatePrevFrames();

    void detectAndCompute();
    void bucketedFeatureDetection();
    void track(cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::KeyPoint>& prev_kps, std::vector<cv::KeyPoint>& cur_kps);
    void trackStereoFeatures();
    
};



void FeatureManager::initiateFrames(cv::Mat image_left, cv::Mat image_right)
{
    _left_cur.image = image_left; 
    _left_cur.descriptor = cv::Mat(); 
    _left_cur.features.clear(); 

    _right_cur.image = image_right; 
    _right_cur.descriptor = cv::Mat(); 
    _right_cur.features.clear(); 
}

void FeatureManager::updatePrevFrames()
{
    _left_prev = _left_cur;
    _right_prev = _right_cur;
}

void FeatureManager::detectAndCompute()  
{        
    _extractor->detect(_left_cur.image, _left_cur.features, cv::Mat()); 
    _extractor->detect(_right_cur.image, _right_cur.features, cv::Mat()); 

    _descriptor->compute(_left_cur.image, _left_cur.features, _left_cur.descriptor); 
    _descriptor->compute(_right_cur.image, _right_cur.features, _right_cur.descriptor); 
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

void FeatureManager::trackStereoFeatures()
{
    track(_left_prev.image, _left_cur.image, _left_prev.features, _left_cur.features);
    track(_right_prev.image, _right_cur.image, _right_prev.features, _right_cur.features);
}