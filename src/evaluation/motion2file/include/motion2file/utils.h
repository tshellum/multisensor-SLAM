#pragma once

/*** Eigen packages ***/
#include <Eigen/Geometry> 

/*** C++ packages ***/
#include <fstream>
#include <sys/stat.h> 


void appendToFile(std::string filepath, double stamp, Eigen::Vector3d t, Eigen::Quaterniond q)
{
    std::ofstream results;
    results.open(filepath, std::ios_base::app);
    results << stamp << " "    // timestamp for current pose
            << t.x() << " "	   // x translation 
            << t.y() << " "	   // y translation
            << t.z() << " "	   // z translation 
            << q.x() << " "	   // x quaternion
            << q.y() << " "	   // y quaternion
            << q.z() << " "	   // y quaternion
            << q.w() << "\n";  // w quaternion

    results.close();
}