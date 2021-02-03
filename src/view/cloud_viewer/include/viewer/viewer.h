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

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


class Visualization
{
private:
	// Point cloud items
	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
	// pcl::visualization::CloudViewer _viewer;
	pcl::visualization::PCLVisualizer _viewer;
  // pcl::visualization::PCLVisualizer::Ptr _viewer;

	int n_id;

	// Pose items
	Eigen::Affine3f    _T_w0;

	Eigen::Matrix3d    _R_wb;
	Eigen::Quaterniond _q_wb;
	Eigen::Vector3d    _t_wb;
	Eigen::Affine3d    _T_wb;
    
public:
	Visualization() : n_id(0),
										_cloud(new pcl::PointCloud<pcl::PointXYZ>),
										_viewer("Cloud Viewer")
										// _viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
	{
		// _t_w0.setZero();
		// _R_w0.setIdentity();
		_T_w0.setIdentity();

		_t_wb.setZero();
		_R_wb.setIdentity();
		_q_wb = _R_wb;
		_T_wb.setIdentity();

		_viewer.setBackgroundColor (1.0, 0.5, 1.0);
		_viewer.addCoordinateSystem (10, _T_w0, "world center", 0);

		// _viewer.addCoordinateSystem (1.0, "world center", 0);   
	};
	~Visualization() {};

	void updatePose(const geometry_msgs::PoseStamped msg);

	void readCloud(const sensor_msgs::PointCloud2 msg);
	void addCamera();
	void setEnvironment();

	bool wasStopped();

	void show();
	void spinOnce();
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


void Visualization::readCloud(const sensor_msgs::PointCloud2 msg) 
{
	pcl::fromROSMsg(msg, *_cloud);

	// _viewer.addPointCloud<pcl::PointXYZ>(_cloud, std::to_string(++n_id));
	// _viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, std::to_string(n_id));

}


void Visualization::addCamera()
{	
	pcl::ModelCoefficients plane_coeff;
	plane_coeff.values.resize (4);    // We need 4 values
	plane_coeff.values[0] = _q_wb.x ();
	plane_coeff.values[1] = _q_wb.y ();
	plane_coeff.values[2] = _q_wb.z ();
	plane_coeff.values[3] = _q_wb.w ();
	
	// _viewer.addPlane(plane_coeff, 1, 1, 1, std::to_string(++n_id));

	Eigen::Affine3f T_wb = _T_wb.cast<float>();
	_viewer.addCoordinateSystem (10, T_wb, std::to_string(++n_id), 0);

	// createViewPort (double xmin, double ymin, double xmax, double ymax, int &viewport)

	// _viewer.addCoordinateSystem (1.0, "camera", 0);   

}


void Visualization::setEnvironment()
{
	// _viewer.setBackgroundColor(1.0, 0.5, 1.0);
}



bool Visualization::wasStopped()
{
	return _viewer.wasStopped();
	// return _viewer->wasStopped();
}



// https://stackoverflow.com/questions/10106288/pcl-visualize-a-point-cloud
// void Visualization::show()
// {
// 	_viewer.showCloud(_cloud);
// }

void Visualization::spinOnce()
{
	_viewer.spinOnce();
}

