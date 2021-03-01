#pragma once 

#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/slam/ProjectionFactor.h> 

#include <array> 
#include <string> 
#include <algorithm> 

#include "apriltag_map/apriltag_map.h"
#include "apriltag_map/ApriltagDetections.h"

#include "backend/backend.h"
#include "backend/factor_handler/factor_handler.h"

namespace backend
{

namespace factor_handler
{

class ApriltagHandler : public FactorHandler<const apriltag_map::ApriltagDetections&>
{
protected: 
  gtsam::Cal3_S2::shared_ptr K_; 
  apriltag_map::ApriltagMap map_; 

  virtual void addApriltagGTInfoToGraph(int id, const apriltag_map::ApriltagGTInfo& info) = 0; 
  virtual void addApriltagDetectionToGraph(const apriltag_map::ApriltagDetection& detection) = 0; 
public: 
  ApriltagHandler(ros::NodeHandle nh, 
    const std::string& apriltag_detections_topic, uint32_t apriltag_detections_queue_size, 
    std::shared_ptr<Backend> backend, 
    const std::string& apriltag_map_filename
  ); 
  ~ApriltagHandler() = default; 

  void callback(const apriltag_map::ApriltagDetections& detections) override; 
}; 

using GTProjectionFactor = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>; 

class ApriltagMultipleGTHandler : public ApriltagHandler
{
private:
  gtsam::noiseModel::Isotropic::shared_ptr measurementNoise_ = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); 

  gtsam::Symbol getApriltagSymbol(int id, std::size_t cornerIndex) const; 

  void addApriltagGTInfoToGraph(int id, const apriltag_map::ApriltagGTInfo& info) override; 
  void addApriltagDetectionToGraph(const apriltag_map::ApriltagDetection& detection) override;
public: 
  ApriltagMultipleGTHandler(
    ros::NodeHandle nh, 
    const std::string& apriltag_detections_topic, uint32_t apriltag_detections_queue_size, 
    std::shared_ptr<Backend> backend, 
    const std::string& apriltag_map_filename
  );
  ~ApriltagMultipleGTHandler() = default; 

  // void callback(const apriltag_map::ApriltagDetections& detections)
  // {
  //   ApriltagHandler::callback(detections); 
  // }
}; 


} // namespace factor_handler


} // namespace backend
