#pragma once

#include <ros/package.h>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <sys/stat.h> 


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