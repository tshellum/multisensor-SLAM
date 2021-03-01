#include "backend/factor_handler/apriltag_handler.h"


namespace backend
{

namespace factor_handler 
{

ApriltagHandler::ApriltagHandler(
  ros::NodeHandle nh, 
  const std::string& apriltag_detections_topic, uint32_t apriltag_detections_queue_size, 
  std::shared_ptr<Backend> backend, 
  const std::string& apriltag_map_filename
) : FactorHandler(nh, 
    apriltag_detections_topic, apriltag_detections_queue_size, 
    backend
  ), map_(apriltag_map_filename) 
{
  std::for_each(map_.begin(), map_.end(),
    [this](const auto& el) { this->addApriltagGTInfoToGraph(el.first, el.second); }
  ); 
}

} // namespace factor_handler

} // namespace backend