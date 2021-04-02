/*** ROS packages ***/
#include <ros/ros.h> 

// Local
#include "backend/factor_handler/factor_handler.h"
#include "backend/backend.h"
#include "backend/factor_handler/gnss_handler.h"
#include "backend/factor_handler/vo_handler.h"
#include "backend/factor_handler/imu_handler.h"

#include <memory> 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "backend_node");
	ROS_INFO("\n\n\n ----- Starting backend ----- \n");

	ros::NodeHandle nh; 

	std::shared_ptr<backend::Backend> backend = std::make_shared<backend::Backend>(); 

	// backend::factor_handler::GNSSHandler gnss(
	// 	nh, "gnss_topic", 1000, 
	// 	backend
	// );

	backend::factor_handler::IMUHandler imu(
		nh, "imu_topic", 1000, 
		backend
	);

	backend::factor_handler::VOHandler vo(
		nh, "vo_topic", 1000, 
		backend
	);

	// backend::factor_handler::ApriltagHandler tag;

	ros::spin();
}

