#pragma once 

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"

#include <vector> 

struct Frame
{
    cv::Mat image; 
    cv::Mat descriptor; 
    std::vector<cv::KeyPoint> keypoints; 
};

