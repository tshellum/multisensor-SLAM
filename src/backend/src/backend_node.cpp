/*** ROS packages ***/
#include <ros/ros.h> 
#include <ros/package.h>

/*** Boost packages ***/
#include <boost/property_tree/ptree.hpp>

/*** Local ***/
#include "backend/factor_handler/factor_handler.h"
#include "backend/backend.h"
#include "backend/factor_handler/vslam_handler.h"
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
	std::string dataset;
	nh.getParam("/dataset", dataset);
	if ( (dataset == "") || argc > 5)
		dataset = argv[1];

	std::string config_path = ros::package::getPath("backend") + "/../../config/" + dataset + "/";
	boost::property_tree::ptree parameters = readConfigFromJsonFile( config_path + "backend.json" );

	std::cout << "Body frame is set to " << parameters.get< std::string >("body_frame") << "..." << std::endl;
	if ( parameters.get< bool >("pgo") )
		std::cout << "Pose graph optimization=ON" << std::endl;
	else
		std::cout << "Pose graph optimization=OFF" << std::endl;

	std::cout << "Enabled sensors:" << std::endl;


	// Initialize backend node
	std::shared_ptr<backend::Backend> backend = std::make_shared<backend::Backend>(parameters); 

	// Global sensor subscribers (if used - shoule be placed first)
	backend::factor_handler::GNSSHandler gnss(
		nh, "gnss_topic", 1000, 
		backend,
		parameters
	);

	// Local/relative sensor subscribers 
	backend::factor_handler::IMUHandler imu(
		nh, "imu_topic", 1000, 
		backend,
		parameters
	);

	backend::factor_handler::VSLAMHandler vslam(
		nh, "vslam_frontend_topic", 1000, 
		backend,
		parameters,
		readConfigFromJsonFile( config_path + "camera.json" )
	);

	backend::factor_handler::LidarOdometryHandler lidar_odometry(
		nh, "lidar_odometry_topic", 1000, 
		backend,
		parameters
	);

	// backend::factor_handler::ApriltagHandler tag;

	std::cout << std::endl;

	ros::spin();
}

