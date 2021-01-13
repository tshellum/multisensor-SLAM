#pragma once 

#include <ros/ros.h> 

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/video/tracking.hpp>

#include <memory> 
#include <vector> 

#include "frame.h"
#include "feature_utils.h"
    

void track(bool isNewKeyframe, Frame previousFrame, Frame& currentFrame, cv::Ptr<cv::Feature2D> detector, int gridSize, int numRetPoints, double tolerance, int trackedThreshold, int& n_id)
{
    Frame newFrame; 
    newFrame.image = currentFrame.image; 
    newFrame.descriptor = cv::Mat(); 

    if (previousFrame.keypoints.empty())
    {
        ROS_INFO("Tracker: Previous has no keypoints - Restart track");
        gridDetectAndCompute(newFrame, detector, gridSize, numRetPoints, tolerance, n_id);
 
        // previousFrame = newFrame; 
        currentFrame = newFrame; 
    }

    // point vectors
    std::size_t nPoints = previousFrame.keypoints.size(); 
    std::vector<cv::Point2f> previousPoints(nPoints), currentPoints(nPoints);

    for (std::size_t i = 0; i < nPoints; ++i)
        previousPoints[i] = previousFrame.keypoints[i].pt; 

    std::vector<uchar> status; 
    std::vector<float> error; 

    // https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
    cv::calcOpticalFlowPyrLK(
        previousFrame.image, 
        newFrame.image, 
        previousPoints, 
        currentPoints, 
        status, 
        error
    );


    std::vector<cv::KeyPoint> good_new;
    for(uint i = 0; i < nPoints; ++i)
    {
        // Select good points
        if ((status[i] == 1) 
            && (currentPoints[i].x > 0) && (currentPoints[i].y > 0)
            && (currentPoints[i].x < previousFrame.image.cols) && (currentPoints[i].y < previousFrame.image.rows)
            ) {
            good_new.push_back(
                cv::KeyPoint(
                    currentPoints[i], 
                    previousFrame.keypoints[i].size, 
                    previousFrame.keypoints[i].angle, 
                    previousFrame.keypoints[i].response, 
                    previousFrame.keypoints[i].octave, 
                    previousFrame.keypoints[i].class_id
                )
            );
        }
    }
    newFrame.keypoints = good_new; 
    

    if (isNewKeyframe || newFrame.keypoints.size() < trackedThreshold)
    {
        gridDetectAndCompute(newFrame, detector, gridSize, numRetPoints, tolerance, n_id);
    }
    else 
    {
        compute(newFrame, detector, n_id);
    }

    currentFrame = newFrame; 
}   


void track(bool isNewKeyframe, Frame previousFrame, Frame& currentFrame, cv::Ptr<cv::GFTTDetector> extractor, cv::Ptr<cv::Feature2D> descriptor, int gridSize, int numRetPoints, double tolerance, int trackedThreshold, int& n_id)
{
    Frame newFrame; 
    newFrame.image = currentFrame.image; 
    newFrame.descriptor = cv::Mat(); 

    if (previousFrame.keypoints.empty())
    {
        ROS_INFO("Tracker: Previous has no keypoints - Restart track");
        gridDetectAndCompute(newFrame, extractor, descriptor, gridSize, numRetPoints, tolerance, n_id);
 
        // previousFrame = newFrame; 
        currentFrame = newFrame; 
    }

    // point vectors
    std::size_t nPoints = previousFrame.keypoints.size(); 
    std::vector<cv::Point2f> previousPoints(nPoints), currentPoints(nPoints);

    for (std::size_t i = 0; i < nPoints; ++i)
        previousPoints[i] = previousFrame.keypoints[i].pt; 

    std::vector<uchar> status; 
    std::vector<float> error; 

    // https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
    cv::calcOpticalFlowPyrLK(
        previousFrame.image, 
        newFrame.image, 
        previousPoints, 
        currentPoints, 
        status, 
        error
    );


    std::vector<cv::KeyPoint> good_new;
    for(uint i = 0; i < nPoints; ++i)
    {
        // Select good points
        if ((status[i] == 1) 
            && (currentPoints[i].x > 0) && (currentPoints[i].y > 0)
            && (currentPoints[i].x < previousFrame.image.cols) && (currentPoints[i].y < previousFrame.image.rows)
            ) {
            good_new.push_back(
                cv::KeyPoint(
                    currentPoints[i], 
                    previousFrame.keypoints[i].size, 
                    previousFrame.keypoints[i].angle, 
                    previousFrame.keypoints[i].response, 
                    previousFrame.keypoints[i].octave, 
                    previousFrame.keypoints[i].class_id
                )
            );
        }
    }
    newFrame.keypoints = good_new; 
    

    if (isNewKeyframe || newFrame.keypoints.size() < trackedThreshold)
    {
        gridDetectAndCompute(newFrame, extractor, descriptor, gridSize, numRetPoints, tolerance, n_id);
    }
    else 
    {
        compute(newFrame, descriptor, n_id);
    }

    currentFrame = newFrame; 
}   


void track(Frame previousFrame, Frame& currentFrame)
{
    Frame newFrame; 
    newFrame.image = currentFrame.image; 
    newFrame.descriptor = cv::Mat(); 

    // point vectors
    std::size_t nPoints = previousFrame.keypoints.size(); 
    std::vector<cv::Point2f> previousPoints(nPoints), currentPoints(nPoints);

    for (std::size_t i = 0; i < nPoints; ++i)
        previousPoints[i] = previousFrame.keypoints[i].pt; 

    std::vector<uchar> status; 
    std::vector<float> error; 

    // https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
    cv::calcOpticalFlowPyrLK(
        previousFrame.image, 
        newFrame.image, 
        previousPoints, 
        currentPoints, 
        status, 
        error
    );


    std::vector<cv::KeyPoint> good_new;
    for(uint i = 0; i < nPoints; ++i)
    {
        // Select good points
        if ((status[i] == 1) 
            && (currentPoints[i].x > 0) && (currentPoints[i].y > 0)
            && (currentPoints[i].x < previousFrame.image.cols) && (currentPoints[i].y < previousFrame.image.rows)
            ) {
            good_new.push_back(
                cv::KeyPoint(
                    currentPoints[i], 
                    previousFrame.keypoints[i].size, 
                    previousFrame.keypoints[i].angle, 
                    previousFrame.keypoints[i].response, 
                    previousFrame.keypoints[i].octave, 
                    previousFrame.keypoints[i].class_id
                )
            );
        }
    }
    newFrame.keypoints = good_new; 
    currentFrame = newFrame; 
}   


void track(Frame previousFrame, Frame& currentFrame, std::vector<cv::Point2f>& prevKps, std::vector<cv::Point2f>& currKps)
{
    Frame newFrame; 
    newFrame.image = currentFrame.image; 
    newFrame.descriptor = cv::Mat(); 

    // point vectors
    std::size_t nPoints = previousFrame.keypoints.size(); 
    std::vector<cv::Point2f> previousPoints(nPoints), currentPoints(nPoints);

    for (std::size_t i = 0; i < nPoints; ++i)
        previousPoints[i] = previousFrame.keypoints[i].pt; 

    std::vector<uchar> status; 
    std::vector<float> error; 

    // https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
    cv::calcOpticalFlowPyrLK(
        previousFrame.image, 
        newFrame.image, 
        previousPoints, 
        currentPoints, 
        status, 
        error
    );


    std::vector<cv::KeyPoint> good_new;
    for(uint i = 0; i < status.size(); ++i)
    {
        // Select good points
        if ((status[i] == 1) 
            && (currentPoints[i].x > 0) && (currentPoints[i].y > 0)
            && (currentPoints[i].x < previousFrame.image.cols) && (currentPoints[i].y < previousFrame.image.rows)
            ) {
            good_new.push_back(
                cv::KeyPoint(
                    currentPoints[i], 
                    previousFrame.keypoints[i].size, 
                    previousFrame.keypoints[i].angle, 
                    previousFrame.keypoints[i].response, 
                    previousFrame.keypoints[i].octave, 
                    previousFrame.keypoints[i].class_id
                )
            );

            prevKps.push_back(previousPoints[i]);
            currKps.push_back(currentPoints[i]);
        }
    }
    newFrame.keypoints = good_new; 
    currentFrame = newFrame; 
}   



void track(cv::Mat prevImg, cv::Mat currImg, std::vector<cv::Point2f>& prevKps, std::vector<cv::Point2f>& currKps)
{
    std::vector<uchar> status; 
    std::vector<float> error; 

    // https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
    cv::calcOpticalFlowPyrLK(
        prevImg, 
        currImg, 
        prevKps, 
        currKps, 
        status, 
        error
    );

    std::vector<cv::Point2f> prevNew;
    std::vector<cv::Point2f> currNew;
    for(uint i = 0; i < status.size(); ++i)
    {
        // Select good points
        if ((status[i] == 1) 
            && (currKps[i].x > 0) && (currKps[i].y > 0)
            && (currKps[i].x < currImg.cols) && (currKps[i].y < currImg.rows)
            ) {
            
            prevNew.push_back(prevKps[i]);
            currNew.push_back(currKps[i]);
            }
    }
    prevKps = prevNew;
    currKps = currKps;
}   



std::vector<cv::DMatch> extractGoodRatioMatches(const std::vector<std::vector<cv::DMatch>>& matches, float max_ratio)
{
  std::vector<cv::DMatch> good_ratio_matches;

  for (const auto& match : matches)
  {
    if (match[0].distance < match[1].distance * max_ratio)
    {
      good_ratio_matches.push_back(match[0]);
    }
  }

  return good_ratio_matches;
}




void extractMatchingPoints(std::vector<cv::DMatch>& matches, 
                std::vector<cv::KeyPoint> prevKeypoints, std::vector<cv::KeyPoint> currKeypoints, 
                std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
{
    // Extract location of good matches
    std::vector<cv::KeyPoint> prevKpts, currKpts; 

    std::vector<int> pointIdxPrev, pointIdxCurr;

    for (auto it = matches.begin(); it != matches.end(); ++it)
    {
        // Get the indexes of the selected matched keypoints
        pointIdxPrev.push_back(it->queryIdx);
        pointIdxCurr.push_back(it->trainIdx);
    }
    
    cv::KeyPoint::convert(prevKeypoints, points1, pointIdxPrev);
    cv::KeyPoint::convert(currKeypoints, points2, pointIdxCurr);
    
}



cv::Mat drawKpts(cv::Mat image, std::vector<cv::Point2f> points)
{
    int r = 3;
    for (int i=0;i<points.size();i++)
        circle(image, cvPoint(points[i].x, points[i].y), r, CV_RGB(255,255,0), 1, 8, 0);

    return image;    
}



cv::Mat drawEpiLines(cv::Mat image, cv::Mat F, int LeftRight, std::vector<cv::Point2f> points)
{
    std::vector<cv::Vec3f> epiLines;
    cv::computeCorrespondEpilines(points, LeftRight, F, epiLines);

    // for all epipolar lines
    for (auto it = epiLines.begin(); it != epiLines.end(); ++it)
    {
        // draw the epipolar line between first and last column
        cv::line(image, cv::Point(0, -(*it)[2] / (*it)[1]),
                cv::Point(image.cols, -((*it)[2] + (*it)[0] * image.cols) / (*it)[1]),
                cv::Scalar(255, 255, 255));
    }

    return image;   
}