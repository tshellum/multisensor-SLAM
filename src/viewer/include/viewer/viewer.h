#pragma once

/*** ROS packages ***/
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>

/*** Eigen packages ***/
#include <Eigen/Dense>

/*** PCL packages ***/
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>


// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/common/common_headers.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>


class Visualization
{
private:
	// Point cloud items
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::visualization::CloudViewer viewer;


	// Pose items
	Eigen::Matrix3d    _R_w0;
	Eigen::Vector3d    _t_w0;

	Eigen::Matrix3d    _R_wb;
	Eigen::Quaterniond _q_wb;
	Eigen::Vector3d    _t_wb;
	Eigen::Affine3d 	 _T_wb;
    
public:
	Visualization() : cloud(new pcl::PointCloud<pcl::PointXYZ>),
										viewer("Cloud Viewer")
	{
		_t_w0.setZero();
		_R_w0.setIdentity();

		_t_wb.setZero();
		_R_wb.setIdentity();
		_q_wb = _R_wb;
		_T_wb.setIdentity();        
	};
	~Visualization() {};

	void updatePose(const geometry_msgs::PoseStamped msg);

	void readCloud(const sensor_msgs::PointCloud2 msg);
	void show();
};


void Visualization::updatePose(const geometry_msgs::PoseStamped msg)
{
	Eigen::Affine3d T_b1b2;
	tf2::fromMsg(msg.pose, T_b1b2);

	_T_wb = _T_wb * T_b1b2;
	_t_wb = _T_wb.translation();
	_R_wb = _T_wb.linear();
	_q_wb = _R_wb;
}


void Visualization::readCloud(const sensor_msgs::PointCloud2 msg) // https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/
{
	// pcl::PCLPointCloud2 pcl_cloud2;
	// pcl_conversions::toPCL(msg, pcl_cloud2);
	// pcl::fromPCLPointCloud2(pcl_cloud2, *cloud);

	pcl::fromROSMsg(msg, *cloud);

	ROS_INFO_STREAM("*cloud: " << *cloud);
}




void Visualization::show()
{
	viewer.showCloud(cloud);
}

