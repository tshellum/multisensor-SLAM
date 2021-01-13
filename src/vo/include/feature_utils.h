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
#include "world_point.h"


void updateIds(Frame& frame, int& n_id)
{
    for (cv::KeyPoint& kp : frame.keypoints) 
    {
        if (kp.class_id == -1)
            kp.class_id = n_id++; 
    }
}


void detect(Frame& frame, cv::Ptr<cv::Feature2D> orb, int& n_id)  
{
    orb->detect(frame.image, frame.keypoints, cv::Mat()); 
    updateIds(frame, n_id);  
}

void compute(Frame& frame, cv::Ptr<cv::Feature2D> orb, int& n_id)
{
    orb->compute(frame.image, frame.keypoints, frame.descriptor); 
    updateIds(frame, n_id);  
}



void orbDetectAndCompute(Frame& frame, cv::Ptr<cv::Feature2D> orb, int& n_id)
{
    detect(frame, orb, n_id); 
    compute(frame, orb, n_id);
}



// std::vector<cv::KeyPoint> sortDecending(std::vector<cv::KeyPoint> keyPoints)
// {
//     //Sorting keypoints by deacreasing order of strength
//     std::vector<float> responseVector;
//     for (unsigned int i =0 ; i<keyPoints.size(); i++) responseVector.push_back(keyPoints[i].response);
//     std::vector<int> Indx(responseVector.size()); std::iota (std::begin(Indx), std::end(Indx), 0);
//     cv::sortIdx(responseVector, Indx, CV_SORT_DESCENDING);
//     std::vector<cv::KeyPoint> keyPointsSorted;
//     for (unsigned int i = 0; i < keyPoints.size(); i++) keyPointsSorted.push_back(keyPoints[Indx[i]]);
    
//     return keyPointsSorted;
// }


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




void gridDetectAndCompute(Frame& frame, cv::Ptr<cv::Feature2D> orb, const int gridSize, int numRetPoints, int tolerance, int& n_id)
{
    std::vector<cv::KeyPoint> subKeypoints; 

    unsigned int nCols = frame.image.cols; 
    unsigned int nRows = frame.image.rows; 

    unsigned int dCols = frame.image.cols / gridSize; 
    unsigned int dRows = frame.image.rows / gridSize; 

    if (!(frame.image.cols % gridSize) && !(frame.image.rows % gridSize))
    {
        for (unsigned int x = 0; x < nCols; x += dCols)
        {
            for (unsigned int y = 0; y < nRows; y += dRows)
            {
                cv::Mat mask = cv::Mat();
                cv::Mat subimg = frame.image(cv::Rect(x, y, dCols, dRows)).clone();
                orb->detect(subimg, subKeypoints, mask); 

                // cv::goodFeaturesToTrack(subimg, subKeypoints, MAX_COUNT, 0.01, 10);

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
        detect(frame, orb, n_id); 
    }
    

    // Non-Max supression 
    // frame.keypoints = sortDecending(frame.keypoints);
    std::sort(frame.keypoints.begin(), frame.keypoints.end(), 
        [](const cv::KeyPoint& kpi, const cv::KeyPoint& kpj){ return kpi.response > kpj.response;  }
    ); 
    frame.keypoints = ssc(frame.keypoints, numRetPoints, tolerance, frame.image.cols, frame.image.rows);        
    


    compute(frame, orb, n_id);

}




void gridDetectAndCompute(Frame& frame, cv::Ptr<cv::GFTTDetector> extractor, cv::Ptr<cv::Feature2D> descriptor, const int gridSize, int numRetPoints, int tolerance, int& n_id)
{
    std::vector<cv::KeyPoint> subKeypoints; 

    unsigned int nCols = frame.image.cols; 
    unsigned int nRows = frame.image.rows; 

    unsigned int dCols = frame.image.cols / gridSize; 
    unsigned int dRows = frame.image.rows / gridSize; 

    if (!(frame.image.cols % gridSize) && !(frame.image.rows % gridSize) || false)
    {
        for (unsigned int x = 0; x < nCols; x += dCols)
        {
            for (unsigned int y = 0; y < nRows; y += dRows)
            {
                cv::Mat mask = cv::Mat();
                cv::Mat subimg = frame.image(cv::Rect(x, y, dCols, dRows)).clone();
                extractor->detect(subimg, subKeypoints, mask); 

                // cv::goodFeaturesToTrack(subimg, subKeypoints, MAX_COUNT, 0.01, 10);

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
        extractor->detect(frame.image, frame.keypoints, cv::Mat()); 
    }
    

    // Non-Max supression 
    // frame.keypoints = sortDecending(frame.keypoints);
    std::sort(frame.keypoints.begin(), frame.keypoints.end(), 
        [](const cv::KeyPoint& kpi, const cv::KeyPoint& kpj){ return kpi.response > kpj.response;  }
    ); 
    frame.keypoints = ssc(frame.keypoints, numRetPoints, tolerance, frame.image.cols, frame.image.rows);        
    

    descriptor->compute(frame.image, frame.keypoints, frame.descriptor); 
}



