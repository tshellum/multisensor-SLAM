#pragma once 

#include <gtsam/geometry/Point3.h> 
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

class ApriltagHandler : public FactorHandler<apriltag_map::ApriltagDetections>
{
protected: 
  gtsam::Cal3_S2::shared_ptr K_; 
  apriltag_map::ApriltagMap map_; 

  virtual void addApriltagGTInfoToGraph(int id, const apriltag_map::ApriltagGTInfo& info) = 0; 
  virtual void addApriltagDetectionToGraph(const apriltag_map::ApriltagDetection& detection) = 0; 
public: 
  ApriltagHandler(
    ros::NodeHandle nh, 
    const std::string& apriltag_detections_topic, uint32_t apriltag_detections_queue_size, 
    std::shared_ptr<Backend> backend, 
    const std::string& apriltag_map_filename
  ); 
  ~ApriltagHandler() = default; 

  void callback(const apriltag_map::ApriltagDetections& detections) override 
  {
    K_ = new gtsam::Cal3_S2(info.K.at(0), info.K.at(4), 0.0, info.K.at(2), info.K.at(5)); 

    std::for_each(detections.detections.begin(), detections.detections.end(),
      [this](const auto& el { this->addApriltagDetectionToGraph(el); })
    ); 
  }
}; 

class ApriltagMultipleGTHandler : public ApriltagHandler
{
private:
  gtsam::Symbol getApriltagSymbol(int id, std::size_t cornerIndex) const 
  {
    return gtsam::symbol_shorthand::A(4 * id + static_cast<int>(cornerIndex)); 
  }

  void addApriltagGTInfoToGraph(int id, const apriltag_map::ApriltagGTInfo& info) override 
  {
    const std::array<gtsam::Point3, 4>& points = info.getPoints(); 

    for (std::size_t i{0}; i < points.size(); ++i)
    {
      gtsam::Point3 point = info.getPose() * points.at(i); 
      gtsam::Symbol currentSymbol = getApriltagSymbol(el.first, i);  
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

  void addApriltagDetectionToGraph(const apriltag_map::ApriltagDetection& detection) override
  {
    for (std::size_t i{0}; i < detection.detectedPoints.size(); ++i)
    {
      gtsam::Point2 pt{
        detection.detectedPoints.at(i).u, 
        detection.detectedPoints.at(i).v
      }; 

      backend_->getGraph().emplace_shared
        <gtsam::GenericProjectionFactor
          <gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
        >(
          pt, gtsam::noiseModel::Isotropic::Sigma(2, 1.0), 
          gtsam::symbol_shorthand::X(backend_->incrementPoseID()), 
          getApriltagSymbol(detection.id, i)
      ); 
    }
  }
public: 
  ApriltagMultipleGTHandler(
    ros::NodeHandle nh, 
    const std::string& apriltag_detections_topic, uint32_t apriltag_detections_queue_size, 
    std::shared_ptr<Backend> backend, 
    const std::string& apriltag_map_filename
  ) : ApriltagHandler(
      ros::NodeHandle nh, 
      const std::string& apriltag_detections_topic, uint32_t apriltag_detections_queue_size, 
      std::shared_ptr<Backend> backend, 
      const std::string& apriltag_map_filename
    )
  {}
  ~ApriltagMultipleGTHandler() = default; 
}; 


} // namespace factor_handler


} // namespace backend
