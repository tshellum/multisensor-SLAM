#include "backend/factor_handler/apriltag_handler.h"


namespace backend
{

namespace factor_handler 
{

ApriltagHandler::ApriltagHandler(ros::NodeHandle nh, 
  const std::string& apriltag_detections_topic, uint32_t apriltag_detections_queue_size, 
  std::shared_ptr<Backend> backend, 
  const std::string& apriltag_map_filename
) : FactorHandler(nh, 
    apriltag_detections_topic, apriltag_detections_queue_size, 
    backend
  ), 
  map_(apriltag_map_filename) 
{
  std::for_each(map_.begin(), map_.end(),
    [this](const apriltag_map::ApriltagMapElement& el) 
    { this->addApriltagGTInfoToGraph(el.first, el.second); }
  ); 
}

void ApriltagHandler::callback(const apriltag_map::ApriltagDetections& detections) 
{
  K_ = gtsam::Cal3_S2::shared_ptr(
    new gtsam::Cal3_S2(
      detections.info.K.at(0), detections.info.K.at(4), 
      0.0, detections.info.K.at(2), detections.info.K.at(5)
    )
  ); 

  std::for_each(detections.detections.begin(), detections.detections.end(),
    [this](const apriltag_map::ApriltagDetection& el)
    { this->addApriltagDetectionToGraph(el); }
  ); 
} 


ApriltagMultipleGTHandler::ApriltagMultipleGTHandler(
  ros::NodeHandle nh, 
  const std::string& apriltag_detections_topic, uint32_t apriltag_detections_queue_size, 
  std::shared_ptr<Backend> backend, 
  const std::string& apriltag_map_filename
) : ApriltagHandler(nh, 
  apriltag_detections_topic, apriltag_detections_queue_size, 
  backend, 
  apriltag_map_filename
)
{}

gtsam::Symbol ApriltagMultipleGTHandler::getApriltagSymbol(int id, std::size_t cornerIndex) const 
{
  return gtsam::symbol_shorthand::A(4 * id + static_cast<int>(cornerIndex)); 
}

void ApriltagMultipleGTHandler::addApriltagGTInfoToGraph(
  int id, const apriltag_map::ApriltagGTInfo& info) 
{
  const std::array<gtsam::Point3, 4>& points = info.getPoints(); 

  for (std::size_t i{0}; i < points.size(); ++i)
  {
    gtsam::Point3 point = info.getPose() * points.at(i); 
    gtsam::Symbol currentSymbol = getApriltagSymbol(id, i);  
    backend_->getValues().insert(
      currentSymbol, 
      point
    ); 
    backend_->getGraph().addPrior(
      currentSymbol, 
      point, 
      gtsam::noiseModel::Isotropic::Sigma(3, 0.1)
    ); 
  }
}

void ApriltagMultipleGTHandler::addApriltagDetectionToGraph(
  const apriltag_map::ApriltagDetection& detection)
{
  for (std::size_t i{0}; i < detection.detectedPoints.size(); ++i)
  {
    gtsam::Point2 pt{
      detection.detectedPoints.at(i).u, 
      detection.detectedPoints.at(i).v
    }; 

    backend_->getGraph().emplace_shared<GTProjectionFactor>(
        pt, measurementNoise_, 
        gtsam::symbol_shorthand::X(backend_->incrementPoseID()), 
        getApriltagSymbol(detection.id, i), 
        K_
    ); 
  }
}

} // namespace factor_handler

} // namespace backend