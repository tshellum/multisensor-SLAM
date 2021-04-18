#pragma once

#include <string>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


#include "DBoW2.h"
#include "thirdparty/TemplatedLoopDetector.h"
#include "FORB.h"


typedef DLoopDetector::TemplatedLoopDetector
  <FORB::TDescriptor, FORB> OrbLoopDetector;


struct LoopResult
{
    bool found;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    int match_id;

  LoopResult()
  : found(false)
  , match_id(-1)
  {}

  LoopResult(bool db_found,
             int db_id,
             std::vector<cv::KeyPoint> db_keypoints,
             std::vector<cv::Mat> db_descriptors)
  : found(db_found), match_id(db_id)
  , keypoints(db_keypoints)
  {
    cv::vconcat(db_descriptors, descriptors);
  }
}; 


class LoopDetector
{
private:
    std::shared_ptr<OrbLoopDetector> loop_detector_;

    int n_keyframes_;

public:
    LoopDetector(
        std::string vocabulary_path, 
        int image_w, int image_h
    )
    : n_keyframes_(0)
    {
        std::string filetype = vocabulary_path.substr(vocabulary_path.size() - 4);
        std::clock_t t_start = std::clock();
        bool b_loaded = true;
        
        OrbVocabulary voc;
        if (filetype == ".bin")
            b_loaded = voc.loadFromBinaryFile(vocabulary_path);  // Waaaaaaay faster than .txt

        else if (filetype == ".txt")
            b_loaded = voc.loadFromTextFile(vocabulary_path);  

        else 
            voc.load(vocabulary_path);
        
        double t_diff = (double)(std::clock() - t_start)/CLOCKS_PER_SEC ;

        if (! b_loaded)
            std::cout << "Wrong path to vocabulary" << std::endl;
        else
            std::cout << "Loaded vocabulary with " << filetype << " extension in " << t_diff << "s" << std::endl;
            
        OrbLoopDetector::Parameters params(image_w, image_h);
        params.use_nss = true;
        params.alpha = 0.9;
        params.k = 1;
        params.geom_check = DLoopDetector::GEOM_DI;
        params.di_levels = 2;

        // GEOM_DI: the direct index is used to select correspondence points between
        //    those features whose vocabulary node at a certain level is the same.
        //    The level at which the comparison is done is set by the parameter
        //    di_levels:
        //      di_levels = 0 -> features must belong to the same leaf (word).
        //         This is the fastest configuration and the most restrictive one.
        //      di_levels = l (l < L) -> node at level l starting from the leaves.
        //         The higher l, the slower the geometrical checking, but higher
        //         recall as well.
        //         Here, L stands for the depth levels of the vocabulary tree.
        //      di_levels = L -> the same as the exhaustive technique.

        loop_detector_.reset(new OrbLoopDetector(voc, params));
     }

    ~LoopDetector() {}

    void changeStructure(const cv::Mat& plain, std::vector<DBoW2::FORB::TDescriptor>& out); 

    LoopResult searchLoopCandidate(std::vector<cv::KeyPoint> keypoints, cv::Mat descriptor);
    
};



void LoopDetector::changeStructure(const cv::Mat& plain, std::vector<DBoW2::FORB::TDescriptor>& out)
{
    out.resize(plain.rows);

    for(int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i);
        // DBoW2::FORB::TDescriptor i_row = plain.row(i);
        // out.push_back(i_row);
    }
}


LoopResult LoopDetector::searchLoopCandidate(std::vector<cv::KeyPoint> keypoints,
                                       cv::Mat descriptor)
{
    n_keyframes_++;

    std::vector<DBoW2::FORB::TDescriptor> desc_restructured;
    changeStructure(descriptor, desc_restructured);
    
    DLoopDetector::DetectionResult result;
    loop_detector_->detectLoop(keypoints, desc_restructured, result);

    if (result.detection())
    {
        if (n_keyframes_ > 50) // More than n keyframes since previous loop closure detection
        {
            n_keyframes_ = 0;
            LoopResult summary = LoopResult(result.detection(),
                                            result.match,
                                            loop_detector_->getKeyPointsAtEntry(result.match),
                                            loop_detector_->getDescriptorsAtEntry(result.match));
            
            return summary;
        }
    }
    return LoopResult();
}

