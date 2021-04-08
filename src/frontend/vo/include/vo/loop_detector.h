#pragma once

// #include "thirdparty/DBoW2/include/DBoW2/DBoW2.h"
#include "thirdparty/DLoopDetector/include/DLoopDetector/DLoopDetector.h"


#include <string>


// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


class LoopDetector
{
private:
    OrbVocabulary   vocabulary_;  // ORB voc: Lw = 6, with 10 nodes each level
    OrbDatabase     db_;
    OrbLoopDetector ld_;
  
    std::vector< std::vector<cv::Mat> > descriptors_;

    int n_keyframes_;

public:
    LoopDetector(int k = 9, int L = 3, DBoW2::WeightingType weight = DBoW2::TF_IDF, DBoW2::ScoringType scoring = DBoW2::L1_NORM)
    : n_keyframes_(0)
    , vocabulary_(k, L, weight, scoring)  //  k^L sized vocabulary
    , db_(vocabulary_, false, 0)          // false = do not use direct index (so ignore the last param)
                                          // The direct index is useful if we want to retrieve the features that belong to some vocabulary node.
                                          // db creates a copy of the vocabulary, we may get rid of "voc" now
    {}

    LoopDetector(std::string vocabulary_path, int image_w, int image_h)
    : n_keyframes_(0)
    {
        std::string filetype = vocabulary_path.substr(vocabulary_path.size() - 3);

        bool b_loaded = true;
        if (filetype == "bin")
            b_loaded = vocabulary_.loadFromBinaryFile(vocabulary_path);  

        else if (filetype == "txt")
            b_loaded = vocabulary_.loadFromTextFile(vocabulary_path);  

        else 
            vocabulary_.load(vocabulary_path); 

        if (! b_loaded)
            ROS_ERROR("Wrong path to vocabulary");
        else
            ROS_INFO_STREAM("Vocabulary with " << filetype << " extension is loaded...");
            
        db_ = OrbDatabase(vocabulary_, false, 0);   // false = do not use direct index (so ignore the last param)
                                                    // The direct index is useful if we want to retrieve the features that belong to some vocabulary node.
                                                    // db creates a copy of the vocabulary, we may get rid of "voc" now

        // Set loop detector parameters
        OrbLoopDetector::Parameters params(image_w, image_h);

        // We are going to change these values individually:
        params.use_nss = true; // use normalized similarity score instead of raw score
        params.alpha = 0.3; // nss threshold
        params.k = 3; // a loop must be consistent with 1 previous matches
        params.geom_check = DLoopDetector::GEOM_DI; // use direct index for geometrical checking
        params.di_levels = 2; // use two direct index levels

        // To verify loops you can select one of the next geometrical checkings:
        // GEOM_EXHAUSTIVE: correspondence points are computed by comparing all
        //    the features between the two images.
        // GEOM_FLANN: as above, but the comparisons are done with a Flann structure,
        //    which makes them faster. However, creating the flann structure may
        //    be slow.
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
        // GEOM_NONE: no geometrical checking is done.
        //
        // In general, with a 10^6 vocabulary, GEOM_DI with 2 <= di_levels <= 4 
        // yields the best results in recall/time.
        // Check the T-RO paper for more information.
        //
        
        // Initiate loop detector with the vocabulary 
        // ld_ = OrbLoopDetector(vocabulary_, params); // TODO: Error....                  
    }

    ~LoopDetector() {}

    void addDescriptor(cv::Mat descriptor); 
    void changeStructure(const cv::Mat& plain, std::vector<cv::Mat>& out);

    void searchLoopCandidate(cv::Mat descriptor);
};



void LoopDetector::addDescriptor(cv::Mat descriptor)
{
    std::vector<cv::Mat> desc_restructured;
    changeStructure(descriptor, desc_restructured);

    descriptors_.push_back(desc_restructured);
    db_.add(desc_restructured);
    n_keyframes_++;
}



void LoopDetector::changeStructure(const cv::Mat& plain, std::vector<cv::Mat>& out)
{
    out.resize(plain.rows);

    for(int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i);
    }
}


void LoopDetector::searchLoopCandidate(cv::Mat descriptor)
{
    std::vector<cv::Mat> desc_restructured;
    changeStructure(descriptor, desc_restructured);
    
    DBoW2::BowVector v1, v2;
    vocabulary_.transform(desc_restructured, v1);
    for(int i = 0; i < n_keyframes_; i++)
    {
        vocabulary_.transform(descriptors_[i], v2);
        
        double score = vocabulary_.score(v1, v2);
        cout << "LOOP CANDIDATE: Image[" << i << "] - Score: " << score << endl;
    }
}
