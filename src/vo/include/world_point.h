#pragma once 

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"

#include "sophus/se3.hpp"

struct WorldPoint
{
    int id; 
    cv::Point3f worldPoint;
    Sophus::SE3d pose; 
};

