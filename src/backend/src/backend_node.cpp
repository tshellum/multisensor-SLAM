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

	ros::NodeHandle pnh("~"); 

	std::shared_ptr<backend::Backend> backend = std::make_shared<backend::Backend>(); 

	std::string detections_topic; 
  int32_t detections_queue_size; 
  pnh.getParam("detections_topic", detections_topic); 
  pnh.getParam("detections_queue_size", detections_queue_size); 

	std::string apriltag_map_filename; 
  pnh.getParam("apriltag_map_filename", apriltag_map_filename); 

	backend::factor_handler::ApriltagMultipleGTHandler apriltag(
		pnh, 
		detections_topic, detections_queue_size, 
		backend, 
		apriltag_map_filename
	);

	std::string gnss_topic; 
	int32_t gnss_queue_size; 
	pnh.getParam("gnss_topic", gnss_topic); 
  pnh.getParam("gnss_queue_size", gnss_queue_size); 
	// backend::factor_handler::GNSSHandler gnss(
	// 	nh, gnss_topic, gnss_queue_size, 
	// 	backend
	// );

	std::string vo_topic; 
	int32_t vo_queue_size; 
	pnh.getParam("vo_topic", vo_topic); 
  pnh.getParam("vo_queue_size", vo_queue_size); 
	// backend::factor_handler::VOHandler vo(
	// 	nh, vo_topic, vo_queue_size, 
	// 	backend
	// );

	// backend::factor_handler::IMUHandler imu;
	// backend::factor_handler::ApriltagHandler tag;

	ros::spin();
}

