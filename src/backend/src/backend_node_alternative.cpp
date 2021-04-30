/*** ROS packages ***/
#include <ros/ros.h> 
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include "backend/VSLAM_msg.h"
#include <sensor_msgs/Imu.h>


/*** GTSAM packages ***/
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h> 
#include <gtsam/geometry/Point3.h> 
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/geometry/Pose3.h> 


/*** Local ***/ 
#include "support.h"

/*** Standard library ***/ 
#include <fstream>
#include <time.h>



class Backend
{
private:
	// Nodes
	ros::NodeHandle nh_;
	ros::Subscriber vo_sub_; 
	ros::Subscriber imu_sub_; 
	ros::Publisher  world_pose_pub_;

	// ISAM2
	int num_opt_;
	gtsam::ISAM2 isam2_; 
	gtsam::ISAM2Params isam2_params_; 
	gtsam::Values new_values_; 
	gtsam::NonlinearFactorGraph new_factors_;
	
	// Current state
	int pose_id_;
	gtsam::Values current_estimate_; 
	gtsam::Pose3 pose_;
	gtsam::Vector3 velocity_;
		

	// VO
	gtsam::noiseModel::Diagonal::shared_ptr pose_noise_; 


	// IMU
	double dt_;
	gtsam::noiseModel::Robust::shared_ptr velocity_noise_model_;
  	gtsam::noiseModel::Robust::shared_ptr bias_noise_model_;
	gtsam::NavState prev_state_;
	gtsam::NavState pred_state_;
	gtsam::imuBias::ConstantBias prev_bias_;
	// gtsam::PreintegratedCombinedMeasurements* preintegrated_;
	std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> preintegrated_;

public:
	Backend(boost::property_tree::ptree config)
	: pose_id_(0)
	, dt_(0.01)
	, num_opt_(20)
	, world_pose_pub_( nh_.advertise<geometry_msgs::PoseStamped>("/backend/pose", 1) ) 
	, vo_sub_( nh_.subscribe("vo_topic", 1000, &Backend::callback, this) )
	, imu_sub_( nh_.subscribe("imu_topic", 1000, &Backend::imu_callback, this) )


	{
		// VO
		pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
			( gtsam::Vector(6) << gtsam::Vector3::Constant(config.get< double >("visual_odometry.orientation_sigma")), 
								gtsam::Vector3::Constant(config.get< double >("visual_odometry.position_sigma"))
			).finished()
		);

		
		gtsam::Pose3 pose_initial_ = gtsam::Pose3::identity();
		if (config != boost::property_tree::ptree())
		{
			Eigen::Quaterniond q(config.get< double >("pose_origin.orientation.w"), 
								config.get< double >("pose_origin.orientation.x"), 
								config.get< double >("pose_origin.orientation.y"), 
								config.get< double >("pose_origin.orientation.z"));
			
			Eigen::Vector3d t(config.get< double >("pose_origin.translation.x"), 
							config.get< double >("pose_origin.translation.y"), 
							config.get< double >("pose_origin.translation.z"));

			pose_initial_ = gtsam::Pose3(gtsam::Rot3(q), gtsam::Point3(t));
		}


		// IMU
		{
			gtsam::noiseModel::Diagonal::shared_ptr gaussian = gtsam::noiseModel::Isotropic::Sigma(6, 0.15);
			bias_noise_model_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);
		}

		{
			gtsam::noiseModel::Isotropic::shared_ptr gaussian = gtsam::noiseModel::Isotropic::Sigma(3, 0.3);
			velocity_noise_model_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);
		}

		// Create preintegrated instance that follows the NED frame
		// boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params;
		// params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(); // ENU
		// // params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(); // NED
		// BodyPtx BodyPty BodyPtz BodyPrx BodyPry BodyPrz AccelerometerSigma GyroscopeSigma IntegrationSigma AccelerometerBiasSigma GyroscopeBiasSigma AverageDeltaT
		// 0 		0 		0 		0 		0 		0 		0.01 			   0.000175 	  0 			   0.000167 			  2.91e-006 		 0.0100395199348279

		// params->accelerometerCovariance = gtsam::I_3x3 * 0.004;  // acc white noise in continuous
		// params->integrationCovariance   = gtsam::I_3x3 * 0.002;  // integration uncertainty continuous
		// params->gyroscopeCovariance     = gtsam::I_3x3 * 0.001;  // gyro white noise in continuous
		// params->biasAccCovariance       = gtsam::I_3x3 * 0.004;  // acc bias in continuous
		// params->biasOmegaCovariance     = gtsam::I_3x3 * 0.001;  // gyro bias in continuous
		// params->biasAccOmegaInt         = gtsam::Matrix::Identity(6, 6) * 1e-4;

		// gtsam::Matrix33 measured_acc_cov = gtsam::I_3x3 * pow(0.01, 2);
		// gtsam::Matrix33 measured_omega_cov = gtsam::I_3x3 * pow(0.000175, 2);
		// gtsam::Matrix33 integration_error_cov = gtsam::I_3x3 * pow(0, 2);

		// auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(); // _D_  for z downwards (as in NED)

		// // auto params = gtsam::PreintegratedImuMeasurements::Params::MakeSharedU(9.81);
		// params->accelerometerCovariance = measured_acc_cov;     // acc white noise in continuous
		// params->integrationCovariance = integration_error_cov;  // integration uncertainty continuous
		// params->gyroscopeCovariance = measured_omega_cov;       // gyro white noise in continuous
		// params->omegaCoriolis = gtsam::Vector3::Zero();

		constexpr double accel_noise_sigma = 7.8480e-04;
		constexpr double gyro_noise_sigma = 1.7500e-04;
		constexpr double accel_bias_rw_sigma = 0.03;
		constexpr double gyro_bias_rw_sigma = 0.0035;
		auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(); // _D_  for z downwards (as in NED)
		params->setAccelerometerCovariance(gtsam::I_3x3 * (accel_noise_sigma * accel_noise_sigma));
		params->setGyroscopeCovariance(gtsam::I_3x3 * (gyro_noise_sigma * gyro_noise_sigma));
		params->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);
		params->setBiasAccCovariance(gtsam::I_3x3 * (accel_bias_rw_sigma * accel_bias_rw_sigma));
		params->setBiasOmegaCovariance(gtsam::I_3x3 * (gyro_bias_rw_sigma * gyro_bias_rw_sigma));
		params->setBiasAccOmegaInt(gtsam::I_6x6 * 1e-5);

		gtsam::Vector3 acc_bias(0.0, 0.0, 0.0);  
		gtsam::Vector3 gyro_bias(0.0, 0.0, 0.0);

		// gtsam::imuBias::ConstantBias prior_imu_bias = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);
		gtsam::imuBias::ConstantBias prior_imu_bias;

		// body to IMU: rotation, translation [meters]
		gtsam::Rot3 R_bi = gtsam::Rot3(gtsam::I_3x3);
		gtsam::Point3 t_bi(0.0, 0.0, 0.0);
		params->body_P_sensor = gtsam::Pose3(R_bi, t_bi);

		gtsam::Vector3 prior_velocity = gtsam::Vector3(0, 0, 0);
		if (config != boost::property_tree::ptree())
			prior_velocity = gtsam::Vector3(config.get< double >("imu.velocity.x"), 
											config.get< double >("imu.velocity.y"), 
											config.get< double >("imu.velocity.z"));

		prev_state_ = gtsam::NavState(pose_initial_, prior_velocity);
		pred_state_ = prev_state_;
		prev_bias_  = prior_imu_bias;

		// preintegrated_ = new gtsam::PreintegratedCombinedMeasurements(params, prior_imu_bias);
	  	preintegrated_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(params, prior_imu_bias);


		// Add priors
		new_values_.insert(gtsam::symbol_shorthand::X(pose_id_), pose_initial_);
		new_values_.insert(gtsam::symbol_shorthand::V(pose_id_), pred_state_.velocity());
		new_values_.insert(gtsam::symbol_shorthand::B(pose_id_), prev_bias_);

		new_factors_.addPrior(gtsam::symbol_shorthand::X(pose_id_), pose_initial_, pose_noise_);
		new_factors_.addPrior(gtsam::symbol_shorthand::V(pose_id_), pred_state_.velocity(), velocity_noise_model_);
		new_factors_.addPrior(gtsam::symbol_shorthand::B(pose_id_), prev_bias_, bias_noise_model_);

		// isam2_.update(new_factors_, new_values_);
	}

	~Backend()
	{
		// delete preintegrated_;
	}


	template <typename FactorType>
	void addFactor(FactorType factor) { new_factors_.add(factor); }

	void imu_callback(const sensor_msgs::ImuConstPtr& msg);
	void callback(const backend::VSLAM_msg msg);
	geometry_msgs::PoseStamped generateMsg();
};


geometry_msgs::PoseStamped Backend::generateMsg()
{
  tf2::Stamped<Eigen::Affine3d> tf2_stamped_T(
    Eigen::Isometry3d{pose_.matrix()}, 
    ros::Time::now(), 
    "backend_pose_world"
  );
  geometry_msgs::PoseStamped stamped_pose_msg = tf2::toMsg(tf2_stamped_T);

  return stamped_pose_msg;
}


void Backend::imu_callback(const sensor_msgs::ImuConstPtr& msg)
  {
    // Read measurements and preintegrate
    Eigen::Vector3d gyr, acc;
    tf2::fromMsg(msg->angular_velocity, gyr);
    tf2::fromMsg(msg->linear_acceleration, acc);
    
    preintegrated_->integrateMeasurement(acc, gyr, dt_);
  }


void Backend::callback(const backend::VSLAM_msg msg)
{ 
    ros::Time tic = ros::Time::now();

	// VO
	Eigen::Isometry3d T_b1b2;
    tf2::fromMsg(msg.pose, T_b1b2);
    gtsam::Pose3 pose_relative(T_b1b2.matrix()); 

	int from_id = pose_id_++;
	gtsam::Key pose_key_from = gtsam::symbol_shorthand::X(from_id); 
    gtsam::Key pose_key_to   = gtsam::symbol_shorthand::X(pose_id_); 

	new_values_.insert(pose_key_to, pose_relative);
    new_factors_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(
        pose_key_from, pose_key_to, pose_relative, pose_noise_
      )
    );


	// IMU
	pred_state_ = preintegrated_->predict(prev_state_, prev_bias_);

    gtsam::Key vel_key_from  = gtsam::symbol_shorthand::V(from_id); 
    gtsam::Key bias_key_from = gtsam::symbol_shorthand::B(from_id); 
    gtsam::Key vel_key_to  = gtsam::symbol_shorthand::V(pose_id_); 
    gtsam::Key bias_key_to = gtsam::symbol_shorthand::B(pose_id_); 

    new_values_.insert(vel_key_to, pred_state_.velocity());
    new_values_.insert(bias_key_to, prev_bias_);

    gtsam::CombinedImuFactor imu_factor(
		pose_key_from, vel_key_from,
		pose_key_to, vel_key_to, 
		bias_key_from, bias_key_to, 
		*preintegrated_
	);

    new_factors_.add(imu_factor);

	ROS_INFO("Optimize");
    // Optimize    
    isam2_.update(new_factors_, new_values_);
    for (int i = 0; i < num_opt_; ++i)
      isam2_.update(); 
    
	ROS_INFO("Optimized");

    // Get updated values
    current_estimate_ = isam2_.calculateBestEstimate();


	// Update IMU states
	pose_ = current_estimate_.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(pose_id_));          // Pose
    velocity_ = current_estimate_.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(pose_id_));  // Velocity
  	prev_bias_ = current_estimate_.at<gtsam::imuBias::ConstantBias>(bias_key_to);

    prev_state_ = gtsam::NavState(pose_,
                                  velocity_);

    preintegrated_->resetIntegrationAndSetBias(prev_bias_);


	// Publish
    world_pose_pub_.publish(generateMsg());

    // Reset parameters
    new_factors_.resize(0);
    new_values_.clear();

    ros::Time toc = ros::Time::now();
    printSummary((toc - tic).toSec(),
                  Eigen::Affine3d{pose_.matrix()} );
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "backend_node");
	ROS_INFO("\n\n\n ----- Starting backend ----- \n");

	// Read initial parameter values
	std::string config_path = ros::package::getPath("backend") + "/../../config/kitti/";
	boost::property_tree::ptree parameters = readConfigFromJsonFile( config_path + "backend/seq00.json" );

	// Initialize backend node
	Backend backend(parameters); 

	ros::spin();
}

