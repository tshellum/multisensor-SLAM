#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include "opencv2/features2d.hpp"

#include "stereo_frontend/support.h"
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
    cv::Ptr<cv::FastFeatureDetector> _extractor;
    // cv::Ptr<cv::GFTTDetector> _extractor; 
    cv::Ptr<cv::Feature2D>    _descriptor; 

    // Detector parameters
    unsigned int _MAX_FEATURES;
    unsigned int _MAX_FEATURES_PATCH;
    int _THRESHOLD;
    
    // Function parameters
    int _WIDTH, _HEIGHT;
    unsigned int _GRID_SIZE; // NxN size grid
    unsigned int _PATCH_W, _PATCH_H;
    unsigned int _N_BUCKETS;

    std::vector< std::vector<cv::KeyPoint> > _feature_buckets;

    // Point parameters    
    int _n_id;


public:
    // FeatureManager(int MAX_FEATURES = 1000, int GRID_SIZE = 20, int THRESHOLD = 25) 
    // : _GRID_SIZE(GRID_SIZE), _MAX_FEATURES(MAX_FEATURES), _THRESHOLD(THRESHOLD),
    //   _n_id(0)
    // {
    //     _extractor = cv::FastFeatureDetector::create(THRESHOLD_, true); //threshold, NMS
    //     // _extractor = cv::GFTTDetector::create(
    //     //                         _MAX_FEATURES, // maximum number of features
    //     //                         0.01,          // quality level
    //     //                         10);           // minimum allowed distance

    //     _descriptor = cv::ORB::create(_MAX_FEATURES);
    // }

    FeatureManager(ros::NodeHandle nh, const std::string name, unsigned int max_features = 1000, int grid_size = 20, int threshold = 25) 
    : _GRID_SIZE(grid_size), _MAX_FEATURES(max_features), _THRESHOLD(threshold),
      _N_BUCKETS(_GRID_SIZE*_GRID_SIZE)
    {
        int img_width_full, img_height_full;
        nh.getParam("/"+name+"/image_width", img_width_full);
        nh.getParam("/"+name+"/image_height", img_height_full);

        _WIDTH = img_width_full - (img_width_full % _GRID_SIZE);
        _HEIGHT = img_height_full - (img_height_full % _GRID_SIZE);

        _PATCH_W = _WIDTH / _GRID_SIZE;
        _PATCH_H = _HEIGHT / _GRID_SIZE;        
        
        _feature_buckets.reserve(_N_BUCKETS);

        _extractor = cv::FastFeatureDetector::create(_THRESHOLD, true); //threshold, NMS
        // _extractor = cv::GFTTDetector::create(
        //                         _MAX_FEATURES, // maximum number of features
        //                         0.01,          // quality level
        //                         10);           // minimum allowed distance

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

    int getGridSize() {return _GRID_SIZE;};
    int getDefaultNorm() {return _descriptor->defaultNorm();};

    void updatePrevFrames();

    void detectAndCompute();
    void bucketedFeatureDetection(bool keep_previous_features);
    void track(cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::KeyPoint>& prev_kps, std::vector<cv::KeyPoint>& cur_kps);
    void track(cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::KeyPoint>& features);
    void trackBuckets();
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



void FeatureManager::bucketedFeatureDetection(bool keep_previous_features = false)
{
    std::vector<cv::KeyPoint> patch_features; 

    int it_bucket = 0;

    for (unsigned int x = 0; x < _WIDTH; x += _PATCH_W)
    {
        for (unsigned int y = 0; y < _HEIGHT; y += _PATCH_H)
        {
            cv::Mat mask = cv::Mat();
            cv::Mat img_patch_cur = _left_cur.image(cv::Rect(x, y, _PATCH_W, _PATCH_H)).clone();
            
            _extractor->detect(img_patch_cur, patch_features, mask); 



            _feature_buckets[it_bucket].insert(std::end(_feature_buckets[it_bucket]), std::begin(patch_features), std::end(patch_features));

            // Correct for patch position
            for (cv::KeyPoint& feature : patch_features)
            {
                feature.pt.x += x;
                feature.pt.y += y; 
                feature.pt.id += _n_id++; 

                _left_cur.features.push_back(feature);
            }
            
            // TODO: Translate from python example: https://github.com/cgarg92/Stereo-visual-odometry/blob/master/Stereo.py
            // if (len(keypoints) > 10):
            //     keypoints = sorted(keypoints, key=lambda x: -x.response)
            //     for kpt in keypoints[0:10]:
            //         kp.append(kpt)

            it_bucket++;
        }
    }

    _descriptor->compute(_left_cur.image, _left_cur.features, _left_cur.descriptor); 
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
        // Select good points - Status variable not enough..
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


void FeatureManager::track(cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::KeyPoint>& features)
{
    // Point vectors
    std::size_t n_pts = features.size(); 
    std::vector<cv::Point2f> prev_features(n_pts), cur_features(n_pts);

    // Convert to Point2f
    for (std::size_t i = 0; i < n_pts; ++i)
        prev_features[i] = features[i].pt; 

    std::vector<uchar> status; 
    std::vector<float> error; 

    // https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
    cv::calcOpticalFlowPyrLK(
        prev_img, 
        cur_img, 
        prev_features, 
        cur_features, 
        status, 
        error
    );


    std::vector<cv::KeyPoint> tracked_kpts;
    for(uint i = 0; i < n_pts; ++i)
    {
        // Select good points
        if (
            (status[i] == 1) 
            && (cur_features[i].x > 0) && (cur_features[i].y > 0)
            && (cur_features[i].x < prev_img.cols) && (cur_features[i].y < prev_img.rows)
        ) 
        {
            tracked_kpts.push_back(
                cv::KeyPoint
                (
                    cur_features[i], 
                    features[i].size, 
                    features[i].angle, 
                    features[i].response, 
                    features[i].octave, 
                    features[i].class_id
                )
            );
        }
    }

    features = tracked_kpts;
}


void FeatureManager::trackBuckets()
{
    std::vector<cv::KeyPoint> total_tracked_features;
    std::vector<cv::KeyPoint> bucket_tracked_features;
    total_tracked_features.reserve(_N_BUCKETS);
    
    unsigned int it_bucket = 0;
    for (unsigned int x = 0; x < _WIDTH; x += _PATCH_W)
    {
        for (unsigned int y = 0; y < _HEIGHT; y += _PATCH_H)
        {
            cv::Mat img_patch_prev = _left_prev.image(cv::Rect(x, y, _PATCH_W, _PATCH_H)).clone();
            cv::Mat img_patch_cur = _left_cur.image(cv::Rect(x, y, _PATCH_W, _PATCH_H)).clone();

            if (! _feature_buckets[it_bucket].empty())
                track(img_patch_prev, img_patch_cur, _feature_buckets[it_bucket]);   

            bucket_tracked_features = _feature_buckets[it_bucket];

            // Correct for patch position
            for (cv::KeyPoint& feature : bucket_tracked_features)
            {
                feature.pt.x += x;
                feature.pt.y += y;                 
            }

            total_tracked_features.insert(std::end(total_tracked_features), std::begin(bucket_tracked_features), std::end(bucket_tracked_features));
            
            it_bucket++;
        }
    }

    _left_cur.features = total_tracked_features;
}



void FeatureManager::trackStereoFeatures()
{
    track(_left_prev.image, _left_cur.image, _left_prev.features, _left_cur.features);
    track(_right_prev.image, _right_cur.image, _right_prev.features, _right_cur.features);
}