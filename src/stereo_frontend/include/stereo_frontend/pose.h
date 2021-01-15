#pragma once

/*** ROS packages ***/
#include <ros/ros.h>
#include <ros/package.h>

/*** C++ packages ***/
#include <fstream>
#include <sys/stat.h> 

/*** Eigen packages ***/
#include <Eigen/Geometry> 


class Pose
{
    private:
        // Global pose
       	cv::Mat t_wb_, R_wb_, euler_wb_;
    	Eigen::Quaterniond q_wb;
        double current_timestamp_;

        // Relative pose
        cv::Mat R_b1b2_, t_b1b2_;
        double scale_;    
        
        // File
        std::string result_path_;
        std::string filename_;
    
    public:
        Pose(std::string filename) 
        : result_path_(ros::package::getPath("stereo_frontend") + "/../../results/"),
          filename_(filename)
        {
            // Create directory
            mkdir(result_path_.c_str(), 0777);

    		// Clear files
            std::ofstream vo_results;
            vo_results.open(result_path_ + filename_, std::ofstream::out | std::ofstream::trunc);
            vo_results.close();

        };

        ~Pose() {};

        void update_pose();

        void pose2file();

};


void Pose::update_pose()
{
    t_wb_ = t_wb_ + R_wb_ * t_b1b2_ * scale_; 
	R_wb_ = R_wb_ * R_b1b2_;
}


void Pose::pose2file()
{
    // Ground truth
    std::ofstream results;
    results.open (result_path_ + filename_, std::ios_base::app);
    results << current_timestamp_  << " "   // timestamp for trajectory
            << t_wb_.at<double>(0) << " "	// x translation 
            << t_wb_.at<double>(1) << " "	// y translation
            << t_wb_.at<double>(2) << " "	// z translation 
            << q_wb.x()		 	   << " "	// x quaternion
            << q_wb.y()		 	   << " "	// y quaternion
            << q_wb.z()		 	   << " "	// y quaternion
            << q_wb.w() 		   << "\n";	// w quaternion

    results.close();
}