#pragma once

#include "thirdparty/DBoW2/include/DBoW2/DBoW2.h"
// #include "thirdparty/DBoW2/include/DBoW2/BowVector.h"

#include <string>


// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


class LoopDetector
{
private:
    OrbVocabulary vocabulary_;  // ORB voc: Lw = 6, with 10 nodes each level
    OrbDatabase db_;

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

    LoopDetector(std::string vocabulary_path)
    : n_keyframes_(0)
    {
        ROS_INFO_STREAM( vocabulary_path.substr(vocabulary_path.size() - 3) );
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
            ROS_INFO("Vocabulary is loaded...");
            
        db_ = OrbDatabase(vocabulary_, false, 0);   // false = do not use direct index (so ignore the last param)
                                                    // The direct index is useful if we want to retrieve the features that belong to some vocabulary node.
                                                    // db creates a copy of the vocabulary, we may get rid of "voc" now
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
