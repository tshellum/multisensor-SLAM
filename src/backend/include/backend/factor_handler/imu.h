#pragma once





#include <sensor_msgs/Imu.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>



struct IMUMeasurement
{
  gtsam::Vector3 gyro;
  gtsam::Vector3 accelerometer;
  double dt;

  IMUMeasurement(){}

  IMUMeasurement(gtsam::Vector3 gyr, gtsam::Vector3 acc)
  : gyro( gyr )
  , accelerometer( acc )
  , dt (0.01)
  {}

  IMUMeasurement(gtsam::Vector3 gyr, gtsam::Vector3 acc, double dt)
  : gyro( gyr )
  , accelerometer( acc )
  , dt(dt)
  {}

};


struct IMUHelper
{
  ros::Subscriber sub_; 

  gtsam::Vector3 prior_velocity;
  gtsam::imuBias::ConstantBias priorImuBias;  // assume zero initial bias
  gtsam::noiseModel::Robust::shared_ptr velocityNoiseModel;
  gtsam::noiseModel::Robust::shared_ptr biasNoiseModel;
  gtsam::NavState prevState;
  gtsam::NavState propState;
  gtsam::imuBias::ConstantBias prevBias;
  gtsam::PreintegratedCombinedMeasurements* preintegrated;

  int from_id;
  bool initialized;
  std::map<ros::Time, IMUMeasurement> stamped_measurements;
  ros::Time prev_stamp;

  IMUMeasurement prev_measurement;

 IMUHelper(ros::NodeHandle nh)
  : sub_( nh.subscribe("imu_topic", 1000, &IMUHelper::callback, this) )
  , from_id(0)
  , initialized(false)
  , prev_stamp(ros::Time(0,0))
  {
    {
      auto gaussian = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << gtsam::Vector3(0.15, 0.15, 0.15), gtsam::Vector3(0.15, 0.15, 0.15))
              .finished());
      auto huber = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);

      biasNoiseModel = huber;
    }

    {
      auto gaussian = gtsam::noiseModel::Isotropic::Sigma(3, 0.3);
      auto huber = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);

      velocityNoiseModel = huber;
    }

    // expect IMU to be rotated in image space co-ords
    auto p = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
        gtsam::Vector3(0.0, 0.0, -9.81));

    // p->accelerometerCovariance =
    //     gtsam::I_3x3 * 0.004;  // acc white noise in continuous
    // p->integrationCovariance =
    //     gtsam::I_3x3 * 0.002;  // integration uncertainty continuous
    // p->gyroscopeCovariance =
    //     gtsam::I_3x3 * 0.001;  // gyro white noise in continuous
    // p->biasAccCovariance = 
    //     gtsam::I_3x3 * 0.004;  // acc bias in continuous
    // p->biasOmegaCovariance =
    //     gtsam::I_3x3 * 0.001;  // gyro bias in continuous
    // p->biasAccOmegaInt = 
    //     gtsam::Matrix::Identity(6, 6) * 1e-4;

    p->accelerometerCovariance =
        gtsam::I_3x3 * pow(0.5, 2);  // acc white noise in continuous
    p->gyroscopeCovariance =
        gtsam::I_3x3 * pow(0.5, 2);  // gyro white noise in continuous
    p->integrationCovariance =
        gtsam::I_3x3 * pow(0.1, 2);  // integration uncertainty continuous
    p->biasAccCovariance = 
        gtsam::I_3x3 * pow(0.1, 2);  // acc bias in continuous
    p->biasOmegaCovariance =
        gtsam::I_3x3 * pow(0.1, 2);  // gyro bias in continuous
    p->biasAccOmegaInt = 
        gtsam::Matrix::Identity(6, 6) * pow(0.1, 2);

    // p->accelerometerCovariance =
    //     gtsam::I_3x3 * pow(0.01, 2);  // acc white noise in continuous
    // p->integrationCovariance =
    //     gtsam::I_3x3 * pow(0.0, 2);  // integration uncertainty continuous
    // p->gyroscopeCovariance =
    //     gtsam::I_3x3 * pow(0.000175, 2);  // gyro white noise in continuous
    // p->biasAccCovariance = 
    //     gtsam::I_3x3 * pow(0.000167, 2);  // acc bias in continuous
    // p->biasOmegaCovariance =
    //     gtsam::I_3x3 * pow(2.91e-6, 2);  // gyro bias in continuous
    // p->biasAccOmegaInt = 
    //     gtsam::Matrix::Identity(6, 6) * pow(0.0, 2);


    gtsam::Pose3 enuTcam = gtsam::Pose3(
      gtsam::Rot3( 0,  0, 1,
                  -1,  0, 0,
                   0, -1, 0), 
      gtsam::Point3(0, 0 , 0));

    gtsam::Pose3 veloTimu = gtsam::Pose3(
      gtsam::Rot3( 9.999976e-01, 7.553071e-04, -2.035826e-03, 
                  -7.854027e-04, 9.998898e-01, -1.482298e-02, 
                   2.024406e-03, 1.482454e-02,  9.998881e-01), 
      gtsam::Point3(-8.086759e-01, 3.195559e-01, -7.997231e-01));

    gtsam::Pose3 camTvelo = gtsam::Pose3(
      gtsam::Rot3( 7.967514e-03, -9.999679e-01, -8.462264e-04, 
                  -2.771053e-03,  8.241710e-04, -9.999958e-01, 
                   9.999644e-01,  7.969825e-03, -2.764397e-03), 
      gtsam::Point3(-1.377769e-02, -5.542117e-02, -2.918589e-01));

    gtsam::Pose3 camTimu = enuTcam *  camTvelo * veloTimu;
    
    // camTimu.print("IMU to CAM (ENU)");

    // gtsam::Rot3 pitch = gtsam::Rot3::Pitch(0.0);
    // gtsam::Rot3 roll = gtsam::Rot3::Roll(0.0);

    // // body to IMU rotation
    // gtsam::Rot3 iRb = pitch * roll;

    // // body to IMU translation (meters)
    // gtsam::Point3 iTb(-1.08, 0.32, -0.72);

    // body in this example is the left camera
    // p->body_P_sensor = gtsam::Pose3(iRb, iTb);
    p->body_P_sensor = camTimu;


    // gtsam::Rot3 prior_rotation = gtsam::Rot3(gtsam::I_3x3);
    // gtsam::Pose3 prior_pose(prior_rotation, gtsam::Point3(0, 0, 0));

    // gtsam::Rot3 prior_rotation = gtsam::Rot3(0.468102, -0.883185, 0.0293924,
    //                                          0.883672, 0.467909, -0.0135848,
    //                                         -0.001755, 0.0323323, 0.999476);
    // gtsam::Point3 prior_pos = gtsam::Point3(12.1457, 21.5035, -0.953975);

    gtsam::Rot3 prior_rotation = gtsam::Rot3(   0.467849, -0.883315,   0.0295488,
                                                0.883787,  0.467808, -0.00869931,
                                             -0.00613896,  0.0301848,   0.999525);
    gtsam::Point3 prior_pos = gtsam::Point3(12.6769, 22.538, -0.950605);

    gtsam::Pose3 prior_pose(prior_rotation, prior_pos);

   

    gtsam::Vector3 acc_bias(0.0, 0.0, 0.0);  
    gtsam::Vector3 gyro_bias(0.0, 0.0, -0.05);

    priorImuBias = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);
    // priorImuBias = gtsam::imuBias::ConstantBias();

    prior_velocity = gtsam::Vector3(   8.9800666354344, 
                                    -0.080196005926315,
                                     0.056655078483203);

    prior_velocity = prior_rotation*prior_velocity;
    prevState = gtsam::NavState(prior_pose, prior_velocity);
    propState = prevState;
    prevBias = priorImuBias;

    preintegrated = new gtsam::PreintegratedCombinedMeasurements(p, priorImuBias);
  }

  ~IMUHelper(){}

  void callback(const sensor_msgs::ImuConstPtr& imu_msg);

  std::map<ros::Time, IMUMeasurement>::iterator integrateMeasurements(ros::Time from_time, ros::Time to_time);
  void clearOldMeasurements(double interval = 2.0);

};

void IMUHelper::callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
  gtsam::Vector3 gyr(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
  gtsam::Vector3 acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);

  IMUMeasurement measurement;
  double dt;
  if ( prev_stamp == ros::Time(0,0) )
  {
    dt = 0.01;
    measurement = IMUMeasurement(gyr, acc, dt);
  }
  else
  {
    dt = (imu_msg->header.stamp - prev_stamp).toSec();

    if ( std::abs(acc.z() - 9.81) > 1.0 )
    {
      ROS_INFO("--- Erronous ---");
      measurement = prev_measurement;
    }
    else
    {
      measurement = IMUMeasurement(gyr, acc, dt);
      prev_measurement = measurement;
    }
  }  

  // preintegrated->integrateMeasurement(acc, gyr, dt);
  stamped_measurements[imu_msg->header.stamp] = measurement;

  prev_stamp = imu_msg->header.stamp;
}



void IMUHelper::clearOldMeasurements(double interval)
{
  if (stamped_measurements.empty())
    return;

  std::map<ros::Time, IMUMeasurement>::iterator stamped_measurement = stamped_measurements.begin();
  ros::Time newest_stamp = stamped_measurements.rbegin()->first;

  while (stamped_measurement != stamped_measurements.end())
  {
    if (stamped_measurement->first > (newest_stamp - ros::Duration(interval)) )
      break;
    
    stamped_measurement = stamped_measurements.erase(stamped_measurement);
  }
}


std::map<ros::Time, IMUMeasurement>::iterator IMUHelper::integrateMeasurements(ros::Time from_time, ros::Time to_time)
{
  std::map<ros::Time, IMUMeasurement>::iterator imu_stamped_measurement = stamped_measurements.begin();
  while ( (imu_stamped_measurement->first <= to_time) && (imu_stamped_measurement != stamped_measurements.end()))
  {
    if (imu_stamped_measurement->first > from_time)      
      preintegrated->integrateMeasurement(imu_stamped_measurement->second.accelerometer, 
                                          imu_stamped_measurement->second.gyro, 
                                          imu_stamped_measurement->second.dt);

    // ROS_INFO_STREAM("IMU measurement - stamp: " << imu_stamped_measurement->first);

    imu_stamped_measurement++;
  }

  return imu_stamped_measurement;
}
