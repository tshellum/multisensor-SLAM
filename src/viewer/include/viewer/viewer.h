#pragma once

/*** ROS packages ***/
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include "viewer/VO_msg.h"

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


// http://www.pcl-users.org/Cloud-Viewer-amp-PCLVisualizer-td4032856.html
// http://www.pcl-users.org/Visualize-PointCloud-in-ROS-failed-td4039937.html
// http://www.pcl-users.org/How-to-draw-camera-pyramids-in-Visualizer-td4019124.html
// https://pcl.readthedocs.io/projects/tutorials/en/latest/pcl_visualizer.html


class Visualization
{
private:
	// Point cloud items
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
	pcl::visualization::PCLVisualizer viewer_;

	int n_id_;

	// Pose items
	Eigen::Affine3f    _T_w0;

	Eigen::Matrix3d    _R_wb;
	Eigen::Quaterniond _q_wb;
	Eigen::Vector3d    _t_wb;
	Eigen::Affine3d    _T_wb;

	Eigen::Affine3d    T_wb_;

public:
	Visualization() : n_id_(0),
					  cloud_(new pcl::PointCloud<pcl::PointXYZ>),
					  viewer_("Cloud Viewer")
	{
		// _t_w0.setZero();
		// _R_w0.setIdentity();
		_T_w0.setIdentity();

		_t_wb.setZero();
		_R_wb.setIdentity();
		_q_wb = _R_wb;
		_T_wb.setIdentity();


		T_wb_.setIdentity();

		viewer_.setBackgroundColor (1.0, 0.5, 1.0);
		viewer_.addCoordinateSystem (2, Eigen::Affine3f::Identity(), "world center", 0);
		// viewer_.setPosition(0, 1000);
		// viewer_.addCoordinateSystem (1.0, "world center", 0);   

	};
	~Visualization() {};

	void readPose(const viewer::VO_msg msg);
	void readCloud(const viewer::VO_msg msg);
	void addCamera();
	void setEnvironment();

	void spinOnce();
	bool wasStopped();
};



void Visualization::readPose(const viewer::VO_msg msg)
{
	Eigen::Affine3d T_b1b2;
	tf2::fromMsg(msg.pose, T_b1b2);

	T_wb_ = T_wb_ * T_b1b2;
}



void Visualization::readCloud(const viewer::VO_msg msg)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	for(int i = 0; i < msg.cloud_size; i++)
	{
		Eigen::Vector3d pt_eig;
		tf2::fromMsg(msg.cloud[i].world_point, pt_eig);

		pt_eig = T_wb_.linear() * pt_eig + T_wb_.translation();

		pcl::PointXYZ pt;
		pt.x = pt_eig.x();
		pt.y = pt_eig.y();
		pt.z = pt_eig.z();

		cloud.points.push_back(pt);
	}

	*cloud_ = cloud;

	viewer_.addPointCloud<pcl::PointXYZ>(cloud_, std::to_string(++n_id_));
	viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, std::to_string(n_id_));
}



void Visualization::addCamera()
{	
	Eigen::Quaterniond q_wb;
	q_wb = T_wb_.linear();

	pcl::ModelCoefficients plane_coeff;
	plane_coeff.values.resize (4);    // We need 4 values
	plane_coeff.values[0] = q_wb.x();
	plane_coeff.values[1] = q_wb.y();
	plane_coeff.values[2] = q_wb.z();
	plane_coeff.values[3] = q_wb.w();
	

	Eigen::Affine3f T_wb = T_wb_.cast<float>();
	viewer_.addCoordinateSystem (1, T_wb, std::to_string(++n_id_), 0);

	// createViewPort (double xmin, double ymin, double xmax, double ymax, int &viewport)

	// viewer_.addCoordinateSystem (1.0, "camera", 0);   

}


void Visualization::setEnvironment()
{
	// viewer_.setBackgroundColor(1.0, 0.5, 1.0);
}



void Visualization::spinOnce()
{
	viewer_.spinOnce();
}



bool Visualization::wasStopped()
{
	return viewer_.wasStopped();
}



