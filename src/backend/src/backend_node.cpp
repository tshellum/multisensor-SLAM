/*** ROS packages ***/
#include <ros/ros.h> 
#include <ros/package.h>

/*** Boost packages ***/
#include <boost/property_tree/ptree.hpp>

/*** Local ***/
#include "backend/factor_handler/factor_handler.h"
#include "backend/backend.h"
#include "backend/factor_handler/vo_handler.h"
#include "backend/factor_handler/lidar_odometry_handler.h"
#include "backend/factor_handler/gnss_handler.h"
#include "backend/factor_handler/imu_handler.h"
#include "support.h"

#include <memory> 
#include <sys/stat.h>
#include <unistd.h>
#include <string>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "backend_node");
	ROS_INFO("\n\n\n ----- Starting backend ----- \n");

	// Read parameters
	ros::NodeHandle nh; 

	// Read initial parameter values
	std::string filename, dataset;
	nh.getParam("/dataset", dataset);
	nh.getParam("/filename", filename);
	if ( (dataset == "") || (filename == "") || argc > 5)
	{
		dataset = argv[1];
		filename = argv[2];
	}

	std::string config_path = ros::package::getPath("backend") + "/../../config/" + dataset + "/";
	boost::property_tree::ptree parameters = readConfigFromJsonFile( config_path + "backend/" + filename + ".json" );


	// Initialize backend node
	std::shared_ptr<backend::Backend> backend = std::make_shared<backend::Backend>(); 


	// Global sensor subscribers (if used - shoule be placed first)
	bool use_gnss = false; // If false: Only used for initialization
	backend::factor_handler::GNSSHandler gnss(
		nh, "gnss_topic", 1000, 
		backend,
		parameters,
		use_gnss
	);

	// Local/relative sensor subscribers 
	backend::factor_handler::IMUHandler imu(
		nh, "imu_topic", 1000, 
		backend,
		parameters
	);

	backend::factor_handler::VOHandler vo(
		nh, "vo_topic", 1000, 
		backend,
		parameters,
		readConfigFromJsonFile( config_path + "camera.json" )
	);

	// backend::factor_handler::LidarOdometryHandler lidar_odometry(
	// 	nh, "lidar_odometry_topic", 1000, 
	// 	backend,
	// 	parameters
	// );

	// backend::factor_handler::ApriltagHandler tag;

	ros::spin();
}

