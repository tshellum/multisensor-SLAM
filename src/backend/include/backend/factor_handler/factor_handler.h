#pragma once 

#include <ros/ros.h> 

#include <memory> 

#include "backend/backend.h"


namespace backend
{

namespace factor_handler
{

template<class ... Types>
class FactorHandler
{
protected:
  ros::NodeHandle nh_; 
  ros::Subscriber sub_; 
  std::shared_ptr<Backend> backend_;
  bool online_; 
public:
  FactorHandler(
    ros::NodeHandle nh, 
    const std::string& topic, uint32_t queue_size, 
    std::shared_ptr<Backend> backend,
    bool sensor_status
  ) 
  : nh_(nh)
  , backend_(backend)
  , online_(sensor_status)
  {
    sub_ = nh.subscribe(topic, queue_size, &FactorHandler::callback, this); 
  }

  FactorHandler(const FactorHandler&) = delete; 
  FactorHandler& operator=(const FactorHandler& other) = delete; 
  
  ~FactorHandler() = default; 

  virtual void callback(Types... args) = 0; 
};

} // namespace factor_handler

} // namespace backend