#pragma once

#include <ros/package.h>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <sys/stat.h> 
#include <iostream>
#include <iomanip>


void NotImplementedError(std::string function_name, std::string filename) // call by NotImplementedError(__func__, __FILE__);
{
  std::vector<std::string> entries;
  boost::split(entries, filename, boost::is_any_of("/"));

  ROS_ERROR_STREAM("Function " << function_name << "() in " << entries.rbegin()[1] << "/" << entries.rbegin()[0] << " not implemented.");
}


boost::property_tree::ptree readConfigFromJsonFile(const std::string& filename) 
{
	boost::property_tree::ptree pt;

	try {
		boost::property_tree::read_json(filename, pt);
	} catch (std::exception& e) {
		std::cerr << "Error Reading File " << filename << ":" << std::endl;
		std::cerr << e.what() << std::endl;
		throw;
	}

	return pt;
}

void printSummary(double tpf,
                  Eigen::Affine3d T_clcr)
{
  Eigen::Vector3d euler = T_clcr.linear().eulerAngles(0, 1, 2);
  Eigen::Vector3d euler_abs = euler.cwiseAbs();

  if ( euler_abs.x() > M_PI / 2 )
    euler.x() = euler_abs.x() - M_PI;

  if ( euler_abs.y() > M_PI / 2 )
    euler.y() = euler_abs.y() - M_PI;

  if ( euler_abs.z() > M_PI / 2 )
    euler.z() = euler_abs.z() - M_PI;
  
  euler *= (180.0/M_PI);

  std::cout << '\r' << std::fixed << std::setprecision(6)
            << "\033[1;33mSUMMARY\033[0m \033[3m(B frame)\033[0m: "
            << "Opt time=" << std::setw(8) << std::setfill(' ') << tpf << "; "
            << std::fixed << std::setprecision(2)
            << "R=" << "["  << std::setw(6) << std::setfill(' ') << euler.x() 
                    << "," << std::setw(6) << std::setfill(' ') << euler.y() 
                    << "," << std::setw(6) << std::setfill(' ') << euler.z() << "]" << "; "
            << "t=" << "["  << std::setw(5) << std::setfill(' ') << T_clcr.translation().x() 
                    << "," << std::setw(5) << std::setfill(' ') << T_clcr.translation().y() 
                    << "," << std::setw(5) << std::setfill(' ') << T_clcr.translation().z() << "]" << "; "
            << std::flush;
}