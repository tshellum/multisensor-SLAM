#pragma once

#include <algorithm> 

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
    int _MAX_FEATURES;
    int _THRESHOLD;
    
    // Function parameters
    int _WIDTH, _HEIGHT;
    unsigned int _GRID_SIZE; // NxN size grid
    unsigned int _PATCH_W, _PATCH_H;
    unsigned int _N_BUCKETS;
    unsigned int _DISTANCE;
    std::vector< std::vector<cv::KeyPoint> > _feature_buckets;

    // Point parameters    
    int _n_id;


public:
    FeatureManager(ros::NodeHandle nh, const std::string name, unsigned int max_features = 10, int grid_size = 20, int threshold = 25, unsigned int distance = 5) 
    : _GRID_SIZE(grid_size), _MAX_FEATURES(max_features), _THRESHOLD(threshold), _DISTANCE(distance),
      _N_BUCKETS(_GRID_SIZE*_GRID_SIZE)
    {
        int img_width_full, img_height_full;
        nh.getParam("/"+name+"/image_width", img_width_full);
        nh.getParam("/"+name+"/image_height", img_height_full);

        _WIDTH = img_width_full - (img_width_full % _GRID_SIZE);
        _HEIGHT = img_height_full - (img_height_full % _GRID_SIZE);

        _PATCH_W = _WIDTH / _GRID_SIZE;
        _PATCH_H = _HEIGHT / _GRID_SIZE;        
        
        _feature_buckets.reserve(_N_BUCKETS); // TODO: Doesn't work?
        

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

    void printNumFeaturesBuckets();

    void updatePrevFrames();

    void detectAndCompute();
    void bucketedFeatureDetection(bool use_existing_features);
    void track(cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::KeyPoint>& prev_kps, std::vector<cv::KeyPoint>& cur_kps);
    void track(cv::Mat prev_img, cv::Mat cur_img, std::vector<cv::KeyPoint>& features);
    void trackBuckets();
    void trackStereoFeatures();

    void circularMatching(std::vector<cv::KeyPoint>& features_left_cur, std::vector<cv::KeyPoint>& features_right_cur);
    
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


void FeatureManager::printNumFeaturesBuckets()
{
    for (unsigned int i = 0; i < _feature_buckets.size(); i++)
        ROS_INFO_STREAM("Bucket number " << i << ". Amount of features: " << _feature_buckets[i].size());
}


void FeatureManager::detectAndCompute()  
{        
    _extractor->detect(_left_cur.image, _left_cur.features, cv::Mat()); 
    _extractor->detect(_right_cur.image, _right_cur.features, cv::Mat()); 

    _descriptor->compute(_left_cur.image, _left_cur.features, _left_cur.descriptor); 
    _descriptor->compute(_right_cur.image, _right_cur.features, _right_cur.descriptor); 
}    



void FeatureManager::bucketedFeatureDetection(bool use_existing_features = false)
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

            // Sort to retain strongest features
            std::sort(patch_features.begin(), patch_features.end(), 
                [](const cv::KeyPoint& kpi, const cv::KeyPoint& kpj){ return kpi.response > kpj.response;  }
            ); 
            
            int max = std::min( _MAX_FEATURES, int(patch_features.size()) );
            patch_features.erase(patch_features.begin()+max,patch_features.end());


            // Initialize buckets 
            // TODO: Simplfy using vec.reserve
            if (_feature_buckets.size() < _N_BUCKETS)
            {
                for (cv::KeyPoint& new_feature : patch_features)
                {   
                    new_feature.class_id += _n_id++; 
                }
                _feature_buckets.push_back(patch_features);

                for (cv::KeyPoint& new_feature : patch_features)
                {   
                    // Correct for patch position
                    new_feature.pt.x += x;
                    new_feature.pt.y += y; 

                    _left_cur.features.push_back(new_feature);
                }
                continue;
            }

            // Retain tracked features and only add strongest features
            // TODO: Check 3x3 frames
            for (cv::KeyPoint& new_feature : patch_features)
            {           
                if (use_existing_features)
                {
                    int x0 = new_feature.pt.x;
                    int y0 = new_feature.pt.y;
    
                    bool skip = false;
                    for (cv::KeyPoint& existing_feature : _feature_buckets[it_bucket]) 
                    {
                        // NMS: Skip point if point is in vicinity of existing points
                        if ( pow((existing_feature.pt.x - x0), 2) + pow((existing_feature.pt.y - y0), 2) < pow(_DISTANCE, 2) )
                        {
                            skip = true;
                            break;
                        }                        
                    }

                    if (skip)
                        continue;
                }

                new_feature.class_id += _n_id++; 
                _feature_buckets[it_bucket].push_back(new_feature);

                // Correct for patch position
                new_feature.pt.x += x;
                new_feature.pt.y += y; 

                _left_cur.features.push_back(new_feature);
            }

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

    std::vector<cv::KeyPoint> tracked_kpts_prev, tracked_kpts_cur;
    for(uint i = 0; i < n_pts; ++i)
    {
        // Select good points - Status variable not enough..
        if (
            (status[i] == 1) 
            && (cur_pts[i].x > 0) && (cur_pts[i].y > 0)
            && (cur_pts[i].x < prev_img.cols) && (cur_pts[i].y < prev_img.rows)
        ) 
        {
            tracked_kpts_prev.push_back(
                cv::KeyPoint
                (
                    prev_pts[i], 
                    prev_kps[i].size, 
                    prev_kps[i].angle, 
                    prev_kps[i].response, 
                    prev_kps[i].octave, 
                    prev_kps[i].class_id
                )
            );
            
            tracked_kpts_cur.push_back(
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
    prev_kps = tracked_kpts_prev;
    cur_kps = tracked_kpts_cur;
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
    if (! _left_prev.features.empty())
    {
        std::vector<cv::KeyPoint> bucket_tracked_features;
        
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
                
                    _left_cur.features.push_back(feature);
                }                
                it_bucket++;
            }
        }
    }
}


void FeatureManager::circularMatching(std::vector<cv::KeyPoint>& features_left_cur, std::vector<cv::KeyPoint>& features_right_cur)
{
    if (! _left_prev.image.empty())
    {
        std::vector<cv::KeyPoint> features_left_prev, features_right_prev;

        // Track in circle
        features_left_cur = _left_cur.features;
        track(_left_cur.image, _left_prev.image, features_left_cur, features_left_prev);
        track(_left_prev.image, _right_prev.image, features_left_prev, features_right_prev);        
        track(_right_prev.image, _right_cur.image, features_right_prev, features_right_cur);
        track(_right_cur.image, _left_cur.image, features_right_cur, features_left_cur);

        int match_nr = 1;
        std::vector<cv::KeyPoint> matched_left, matched_right;
        for(unsigned int i = 0; i < features_left_cur.size(); i++)
        {
            for (cv::KeyPoint& original_feature : _left_cur.features)
            {
                if (features_left_cur[i].class_id != original_feature.class_id)
                    continue;

                if ( (( features_left_cur[i].pt.x - original_feature.pt.x) < 1) 
                   && (( features_left_cur[i].pt.y == original_feature.pt.y) < 1) )
                {
                    matched_left.push_back(features_left_cur[i]);
                    matched_right.push_back(features_right_cur[i]);

                    break;
                }
            }
            match_nr++;
        }
        features_left_cur = matched_left;
        features_right_cur = matched_right;
    }
}

