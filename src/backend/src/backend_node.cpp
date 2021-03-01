/*** ROS packages ***/
#include <ros/ros.h> 

#include <string> 

// Local
#include "backend/backend.h"
#include "backend/factor_handler/factor_handler.h"
// #include "backend/factor_handler/gnss_handler.h"
// #include "backend/factor_handler/vo_handler.h"
#include "backend/factor_handler/apriltag_handler.h"

#include <memory> 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "backend_node");
	ROS_INFO("\n\n\n ----- Starting backend ----- \n");

	ros::NodeHandle nh; 

	std::shared_ptr<backend::Backend> backend = std::make_shared<backend::Backend>(); 

	std::string detections_topic; 
  int32_t detections_queue_size; 
  nh.getParam("detections_topic", detections_topic); 
  nh.getParam("detections_queue_size", detections_queue_size); 

	std::string apriltag_map_filename; 
  nh.getParam("apriltag_map_filename", apriltag_map_filename); 

	backend::factor_handler::ApriltagMultipleGTHandler apriltag(
		nh, 
		detections_topic, detections_queue_size, 
		backend, 
		apriltag_map_filename
	);

	// backend::factor_handler::GNSSHandler gnss(
	// 	nh, "gnss_topic", 1, 
	// 	backend
	// );

	// backend::factor_handler::VOHandler vo(
	// 	nh, "vo_topic", 1, 
	// 	backend
	// );

	// backend::factor_handler::IMUHandler imu;
	// backend::factor_handler::ApriltagHandler tag;

	ros::spin();
}

