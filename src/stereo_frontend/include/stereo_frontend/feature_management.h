#pragma once

#include <ros/ros.h> 

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"

#include <algorithm> 
#include <vector> 
#include <memory> 
#include <numeric>

#include "frame.h"

class FeatureManager
{
private:
    cv::Ptr<cv::GFTTDetector> extractor_; 
    // cv::Ptr<cv::FastFeatureDetector> extractor_;
    cv::Ptr<cv::Feature2D> descriptor_; 
    cv::Ptr<cv::Feature2D> detector_; 

    int MAX_FEATURES_;
    int THRESHOLD_;
    int NUM_RETURN_POINTS_; //choose exact number of return points
    float TOLERANCE_;
    unsigned int GRID_SIZE_; // 4x4 size grid

    int n_id = 0; 

public:
    FeatureManager(int MAX_FEATURES = 1000, int GRID_SIZE = 20, int THRESHOLD = 25, int NUM_RETURN_POINTS = 1500, int TOLERANCE = 0.1) 
            : GRID_SIZE_(GRID_SIZE), MAX_FEATURES_(MAX_FEATURES), THRESHOLD_(THRESHOLD),
		    NUM_RETURN_POINTS_(NUM_RETURN_POINTS), TOLERANCE_(TOLERANCE)
    {
        extractor_ = cv::GFTTDetector::create(
                                MAX_FEATURES_, // maximum number of keypoints
                                0.01,          // quality level
                                10);           // minimum allowed distance

        // extractor_ = cv::FastFeatureDetector::create(THRESHOLD_, true); //threshold, NMS
        descriptor_ = cv::ORB::create(MAX_FEATURES_);
        detector_ = cv::ORB::create(MAX_FEATURES_); 
    }
    ~FeatureManager() {} 

    
    int getDefaultNorm()
    {
        return descriptor_->defaultNorm();
    }

    
    void updateIds(Frame& frame)
    {
        for (cv::KeyPoint& kp : frame.keypoints) 
        {
            if (kp.class_id == -1)
                kp.class_id = n_id++; 
        }
    }

    void detectAndCompute(
        Frame& frame,
        bool useProvidedKeypoints=false)  
    {        
        detector_->detectAndCompute(frame.image, cv::Mat(), frame.keypoints, frame.descriptor); 
        // updateIds(frame);  
    }

    void detect(Frame& frame)  
    {
        extractor_->detect(frame.image, frame.keypoints, cv::Mat()); 
        // updateIds(frame);  
    }

    void nmsDetect(Frame& frame)  
    {
        extractor_->detect(frame.image, frame.keypoints, cv::Mat());
        
        // Non-Max supression 
        std::sort(frame.keypoints.begin(), frame.keypoints.end(), 
            [](const cv::KeyPoint& kpi, const cv::KeyPoint& kpj){ return kpi.response > kpj.response;  }
        ); 
        frame.keypoints = ssc(frame.keypoints, NUM_RETURN_POINTS_, TOLERANCE_, frame.image.cols, frame.image.rows);         
        // updateIds(frame);  
    }

    void compute(Frame& frame)
    {
        detector_->compute(frame.image, frame.keypoints, frame.descriptor); 
        // updateIds(frame);  
    }

    void gridDetectAndCompute(Frame& frame)
    {
        std::vector<cv::KeyPoint> subKeypoints; 

        unsigned int nCols = frame.image.cols; 
        unsigned int nRows = frame.image.rows; 

        unsigned int dCols = frame.image.cols / GRID_SIZE_; 
        unsigned int dRows = frame.image.rows / GRID_SIZE_; 

        if (!(frame.image.cols % GRID_SIZE_) && !(frame.image.rows % GRID_SIZE_) || false)
        {
            for (unsigned int x = 0; x < nCols; x += dCols)
            {
                for (unsigned int y = 0; y < nRows; y += dRows)
                {
                    cv::Mat mask = cv::Mat();
                    cv::Mat subimg = frame.image(cv::Rect(x, y, dCols, dRows)).clone();
                    extractor_->detect(subimg, subKeypoints, mask); 

                    for (cv::KeyPoint& kp : subKeypoints)
                    {
                        kp.pt.x += x;
                        kp.pt.y += y; 
                        
                        frame.keypoints.push_back(kp);
                    }
                }
            }
        }
        else
        {
            ROS_INFO("NOT VALID DIMENTIONS");
            extractor_->detect(frame.image, frame.keypoints, cv::Mat()); 
        }
        

        // Non-Max supression 
        std::sort(frame.keypoints.begin(), frame.keypoints.end(), 
            [](const cv::KeyPoint& kpi, const cv::KeyPoint& kpj){ return kpi.response > kpj.response;  }
        ); 
        frame.keypoints = ssc(frame.keypoints, NUM_RETURN_POINTS_, TOLERANCE_, frame.image.cols, frame.image.rows);        
        

        descriptor_->compute(frame.image, frame.keypoints, frame.descriptor);    
    }



    // Non-max supression
    // https://github.com/BAILOOL/ANMS-Codes
    std::vector<cv::KeyPoint> ssc(std::vector<cv::KeyPoint> keyPoints, int numRetPoints,float tolerance, int cols, int rows)
    {
        // several temp expression variables to simplify solution equation
        int exp1 = rows + cols + 2*numRetPoints;
        long long exp2 = ((long long) 4*cols + (long long)4*numRetPoints + (long long)4*rows*numRetPoints + (long long)rows*rows + (long long) cols*cols - (long long)2*rows*cols + (long long)4*rows*cols*numRetPoints);
        double exp3 = sqrt(exp2);
        double exp4 = numRetPoints - 1;

        double sol1 = -round((exp1+exp3)/exp4); // first solution
        double sol2 = -round((exp1-exp3)/exp4); // second solution

        int high = (sol1>sol2)? sol1 : sol2; //binary search range initialization with positive solution
        int low = floor(sqrt((double)keyPoints.size()/numRetPoints));

        int width;
        int prevWidth = -1;
        std::vector<int> ResultVec;
        bool complete = false;
        unsigned int K = numRetPoints; unsigned int Kmin = round(K-(K*tolerance)); unsigned int Kmax = round(K+(K*tolerance));


        std::vector<int> result; result.reserve(keyPoints.size());
        while(!complete){

            width = low+(high-low)/2;
            if (width == prevWidth || low>high || width < 2) { //needed to reassure the same radius is not repeated again
                ResultVec = result; //return the keypoints from the previous iteration
                break;
            }

            result.clear();            
            double c = width/2; //initializing Grid
            int numCellCols = floor(cols/c);
            int numCellRows = floor(rows/c);
            std::vector<std::vector<bool> > coveredVec(numCellRows+1,std::vector<bool>(numCellCols+1,false));
            
            // ROS_INFO("SSC: before first for in while");
            for (unsigned int i=0;i<keyPoints.size();++i){
                int row = floor(keyPoints[i].pt.y / c); //get position of the cell current point is located at
                int col = floor(keyPoints[i].pt.x / c);
                // row = std::min( row, int(coveredVec.size()-1) ); row = std::max(row, 0);

                if (coveredVec[row][col]==false){ // if the cell is not covered
                    result.push_back(i);
                    int rowMin = ((row-floor(width/c))>=0)? (row-floor(width/c)) : 0; //get range which current radius is covering
                    int rowMax = ((row+floor(width/c))<=numCellRows)? (row+floor(width/c)) : numCellRows;
                    int colMin = ((col-floor(width/c))>=0)? (col-floor(width/c)) : 0;
                    int colMax = ((col+floor(width/c))<=numCellCols)? (col+floor(width/c)) : numCellCols;
                    for (int rowToCov=rowMin; rowToCov<=rowMax; ++rowToCov){
                        for (int colToCov=colMin ; colToCov<=colMax; ++colToCov){
                            if (!coveredVec[rowToCov][colToCov]) coveredVec[rowToCov][colToCov] = true; //cover cells within the square bounding box with width w
                        }
                    }
                }
            }

            if (result.size()>=Kmin && result.size()<=Kmax){ //solution found
                ResultVec = result;
                complete = true;
            }
            else if (result.size()<Kmin) high = width-1; //update binary search range
            else low = width+1;
            prevWidth = width;


        }

        // retrieve final keypoints
        std::vector<cv::KeyPoint> kp;
        for (unsigned int i = 0; i<ResultVec.size(); i++) kp.push_back(keyPoints[ResultVec[i]]);

        return kp;
    }

};




