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

  LoopResult()
  : found(false)
  {}

  LoopResult(bool db_found,
             std::vector<cv::KeyPoint> db_keypoints,
             std::vector<cv::Mat> db_descriptors)
  : found(db_found)
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

        ROS_INFO_STREAM("Loading vocabulary with " << filetype << " extension...");
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
            ROS_ERROR("Wrong path to vocabulary");
        else
            ROS_INFO_STREAM("Vocabulary loaded in " << t_diff << "s");
            
        
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
                                            loop_detector_->getKeyPointsAtEntry(result.match),
                                            loop_detector_->getDescriptorsAtEntry(result.match));
            
            return summary;
        }
    }
    return LoopResult();
}






















// class LoopDetector
// {
// private:
//     OrbVocabulary   vocabulary_;  // ORB voc: Lw = 6, with 10 nodes each level
//     OrbDatabase     db_;
//     OrbLoopDetector ld_;
  
//     std::vector< std::vector<cv::Mat> > descriptors_;

//     int n_keyframes_;

// public:
//     LoopDetector(
//         std::string vocabulary_path, 
//         int image_w, int image_h
//     )
//     : n_keyframes_(0)
//     {
//         std::string filetype = vocabulary_path.substr(vocabulary_path.size() - 4);

//         ROS_INFO_STREAM("Loading vocabulary with " << filetype << " extension...");
//         std::clock_t t_start = std::clock();
//         bool b_loaded = true;
        
//         if (filetype == ".bin")
//             b_loaded = vocabulary_.loadFromBinaryFile(vocabulary_path);  // Waaaaaaay faster than .txt

//         else if (filetype == ".txt")
//             b_loaded = vocabulary_.loadFromTextFile(vocabulary_path);  

//         else 
//             vocabulary_.load(vocabulary_path);
        
//         double t_diff = (double)(std::clock() - t_start)/CLOCKS_PER_SEC ;

//         if (! b_loaded)
//             ROS_ERROR("Wrong path to vocabulary");
//         else
//             ROS_INFO_STREAM("Vocabulary loaded in " << t_diff << "s");
            
//         db_ = OrbDatabase(vocabulary_, DLoopDetector::GEOM_DI, 2);

//         // Set loop detector parameters
//         OrbLoopDetector::Parameters params(image_w, image_h);

//         // We are going to change these values individually:
//         params.use_nss = true; // use normalized similarity score instead of raw score
//         params.alpha = 0.3; // nss threshold
//         params.k = 2; // a loop must be consistent with 1 previous matches
//         params.geom_check = DLoopDetector::GEOM_DI; // use direct index for geometrical checking
//         params.di_levels = 2; // use two direct index levels

//         // To verify loops you can select one of the next geometrical checkings:
//         // GEOM_EXHAUSTIVE: correspondence points are computed by comparing all
//         //    the features between the two images.
//         // GEOM_FLANN: as above, but the comparisons are done with a Flann structure,
//         //    which makes them faster. However, creating the flann structure may
//         //    be slow.
//         // GEOM_DI: the direct index is used to select correspondence points between
//         //    those features whose vocabulary node at a certain level is the same.
//         //    The level at which the comparison is done is set by the parameter
//         //    di_levels:
//         //      di_levels = 0 -> features must belong to the same leaf (word).
//         //         This is the fastest configuration and the most restrictive one.
//         //      di_levels = l (l < L) -> node at level l starting from the leaves.
//         //         The higher l, the slower the geometrical checking, but higher
//         //         recall as well.
//         //         Here, L stands for the depth levels of the vocabulary tree.
//         //      di_levels = L -> the same as the exhaustive technique.
//         // GEOM_NONE: no geometrical checking is done.
//         //
//         // In general, with a 10^6 vocabulary, GEOM_DI with 2 <= di_levels <= 4 
//         // yields the best results in recall/time.
//         // Check the T-RO paper for more information.
//         //
        
//         // Initiate loop detector with the vocabulary 
//         ld_ = OrbLoopDetector(&db_, params);      
//         // ld_ = OrbLoopDetector(db_, params);  
//         // ld_.allocate(1000);    
//      }

//     ~LoopDetector() {}

//     void addDescriptor(cv::Mat descriptor); 
//     void changeStructure(const cv::Mat& plain, std::vector<DBoW2::FORB::TDescriptor>& out); 

//     void searchLoopCandidate(std::vector<cv::KeyPoint> keypoints, cv::Mat descriptor);
    
// };



// void LoopDetector::addDescriptor(cv::Mat descriptor)
// {
//     std::vector<cv::Mat> desc_restructured;
//     changeStructure(descriptor, desc_restructured);

//     descriptors_.push_back(desc_restructured);
//     // db_.add(desc_restructured);
//     n_keyframes_++;
// }



// void LoopDetector::changeStructure(const cv::Mat& plain, std::vector<DBoW2::FORB::TDescriptor>& out)
// {
//     out.resize(plain.rows);

//     for(int i = 0; i < plain.rows; ++i)
//     {
//         out[i] = plain.row(i);
//         // DBoW2::FORB::TDescriptor i_row = plain.row(i);
//         // out.push_back(i_row);
//     }
// }



// void LoopDetector::searchLoopCandidate(std::vector<cv::KeyPoint> keypoints,
//                                        cv::Mat descriptor)
// {

//     std::vector<DBoW2::FORB::TDescriptor> desc_restructured;
//     changeStructure(descriptor, desc_restructured);
    
//     BowVector bowvec;
//     FeatureVector featvec;

//     // vocabulary_.transform(desc_restructured, v1);
//     // vocabulary_.transform(desc_restructured, bowvec, featvec, 2);

//     DLoopDetector::DetectionResult result;
//     ROS_INFO_STREAM("Looking for loop, kpts: " << keypoints.size() << ", desc: " << descriptor.size() << ", desc restr: " << desc_restructured.size());
//     const OrbVocabulary* voc = db_.getVocabulary();
//     ROS_INFO("gotten voc");
//     ROS_INFO_STREAM("vocabulary_.size(): " << vocabulary_.size());
//     ROS_INFO_STREAM("voc.size(): " << voc->size());
//     ROS_INFO_STREAM("db_.size(): " << db_.size());
    
//     const OrbVocabulary* voc2 = ld_.getDatabase().getVocabulary();
//     ROS_INFO_STREAM("voc.size(): " << voc2->size());
    
//     // ld_.detectLoop(keypoints, desc_restructured, result);
//     ROS_INFO("Looked for loop");

//     // ROS_INFO_STREAM("Loop detection? : " << result.detection());
// }
