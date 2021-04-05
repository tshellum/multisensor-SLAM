// Tilsvarer example.cpp

#include <ros/ros.h>
// PCL specific includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/features/usc.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

#include <boost/property_tree/ptree.hpp>

#include <Eigen/Dense> 

#include "inputOutput.h"
#include "support.h"

#include <math.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <fstream>
#include <iostream>

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;
ros::Publisher pub6;
ros::Publisher pub7;
ros::Publisher pub8;
ros::Publisher pub9;
ros::Publisher pub10;
ros::Publisher pub11;
ros::Publisher pose_pub;


ofstream file;
ofstream odom_file;

ros::Time stamp_prev_measurement;
int iss_points=0;
int narf_points=0;
int siftz_points=0;
int counter=1;
int counter2=1;
int slam_counter=1;
int map_counter=0;

int total_cor = 0;
int total_points = 0;
int total_clouds = 0;


double distance_since_last = 0.0;
double total_distance = 0.0;
double prev_north = -0.65;
double prev_east = 2.8;
bool send_message = false;

geometry_msgs::PointStamped point_msg;
geometry_msgs::Point point; 
pcl::PointCloud<pcl::PointXYZI>::Ptr source_cor (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr loop (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr loop2 (new pcl::PointCloud<pcl::PointXYZI> ());

Eigen::Vector3f nav_trans;
Eigen::Matrix3f nav_rot;
Eigen::Vector3f map_translation;
Eigen::Vector3f map_total_translation;


pcl::PointCloud<pcl::PointXYZI>::Ptr map (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr current_map (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr loop_closure_map (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr loop_closure_map2 (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::SHOT352>::Ptr feature_map (new pcl::PointCloud<pcl::SHOT352>);


pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr loop_keypoints (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr trans_keypoints (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_loop_keypoints (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr later_loop_keypoints (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr full_map (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::SHOT352>::Ptr source_cloud_keypoints (new pcl::PointCloud<pcl::SHOT352>);
pcl::PointCloud<pcl::SHOT352>::Ptr target_cloud_keypoints (new pcl::PointCloud<pcl::SHOT352>);
pcl::PointCloud<pcl::SHOT352>::Ptr loop_cloud_keypoints (new pcl::PointCloud<pcl::SHOT352>);
pcl::PointCloud<pcl::SHOT352>::Ptr filtered_loop_cloud_keypoints (new pcl::PointCloud<pcl::SHOT352>);
pcl::PointCloud<pcl::SHOT352>::Ptr later_loop_cloud_keypoints (new pcl::PointCloud<pcl::SHOT352>);

Eigen::Matrix4f since_last_transformation; 
Eigen::Matrix3f total_rotation;
Eigen::Vector3f total_translation;
Eigen::Matrix3f since_last_rotation;
Eigen::Vector3f since_last_translation;

//Brukes til ISS keypoints
double
computeCloudResolution (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<pcl::PointXYZI> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

//Brukes til ISS keypoints
double iss_gamma_21_ (0.975); 
double iss_gamma_32_ (0.975);
double iss_min_neighbors_ (4);
int iss_threads_ (4);

void iss(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud_ptr){
    double model_resolution = computeCloudResolution(point_cloud_ptr);
    double iss_salient_radius_(6*model_resolution);
    double iss_non_max_radius_(4*model_resolution);
    double iss_normal_radius_(4*model_resolution);
    double iss_border_radius_(model_resolution);


    pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZI> ());

    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud (point_cloud_ptr);
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (3);
    ne.compute (*cloud_normals);

    //Henter ISS keypoints
    pcl::ISSKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> iss_detector;
    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (iss_salient_radius_);
    iss_detector.setNonMaxRadius (iss_non_max_radius_);

    iss_detector.setNormalRadius (iss_normal_radius_);
    iss_detector.setBorderRadius (iss_border_radius_);

    iss_detector.setThreshold21 (iss_gamma_21_);
    iss_detector.setThreshold32 (iss_gamma_32_);
    iss_detector.setMinNeighbors (iss_min_neighbors_);
    iss_detector.setNumberOfThreads (iss_threads_);
    iss_detector.setInputCloud (point_cloud_ptr);
    iss_detector.setNormals (cloud_normals);
    iss_detector.compute (*model_keypoints);

    source_keypoints = model_keypoints;
    source_cloud = point_cloud_ptr;

    model_keypoints->header.frame_id = point_cloud_ptr->header.frame_id;
    model_keypoints->header.stamp = point_cloud_ptr->header.stamp;

    pcl::PCLPointCloud2 pcl_pc;
    pcl::toPCLPointCloud2(*model_keypoints, pcl_pc);

    pub4.publish (pcl_pc);

    /* SHOT estimation part */
    pcl::SHOTEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::SHOT352> shot;
    shot.setInputCloud (model_keypoints);
    shot.setInputNormals (cloud_normals);
    shot.setSearchMethod (tree);
    shot.setSearchSurface (point_cloud_ptr);
    pcl::PointCloud<pcl::SHOT352>::Ptr shotFeatures (new pcl::PointCloud<pcl::SHOT352> ());
    shot.setRadiusSearch (4);
    shot.compute (*shotFeatures);

    //Setter descriptorer som får NaN til 0
    for (size_t i = 0; i < shotFeatures->size (); ++i)
    {
      if (std::isnan(shotFeatures->points[i].descriptor[0])){
        for (size_t j = 0; j<352; ++j){
          shotFeatures->points[i].descriptor[j]=0;
        }
      }
    } 

    source_cloud_keypoints = shotFeatures;

    //Finner correspondences basert på descriptorer
    boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> corr_est;
    corr_est.setInputSource (source_cloud_keypoints);
    corr_est.setInputTarget (target_cloud_keypoints);
    corr_est.determineReciprocalCorrespondences (*correspondences, 0.8);

    //Fjerner correspondences som ligger for langt unna hverandre
    boost::shared_ptr<pcl::Correspondences> new_correspondences (new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorDistance rej;
    rej.setInputSource<pcl::PointXYZI> (source_keypoints);
    rej.setInputTarget<pcl::PointXYZI> (target_keypoints);
    rej.setMaximumDistance (10);    // 10m
    rej.setInputCorrespondences (correspondences);
    rej.getCorrespondences (*new_correspondences);

    //RANSAC på correspondences som er igjen for å finne noe som er consistent
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> correspondence_rejector;
    correspondence_rejector.setInputSource(source_keypoints);
    correspondence_rejector.setInputTarget(target_keypoints);
    correspondence_rejector.setInlierThreshold(0.2);
    correspondence_rejector.setMaximumIterations(800);
    correspondence_rejector.setRefineModel(true);
    correspondence_rejector.setInputCorrespondences(new_correspondences);

    boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr (new pcl::Correspondences);
    correspondence_rejector.getCorrespondences(*cor_inliers_ptr);

    //Regner ut transformation matrix basert på RANSAC
    Eigen::Matrix4f transformation; 
    transformation = correspondence_rejector.getBestTransformation();

    Eigen::Matrix3f current_rotation;
    current_rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::Vector3f current_translation;
    current_translation << 0, 0, 0;

    Eigen::Vector3f translation;
    Eigen::Matrix3f rotation;

    if (counter != 0){
      if (cor_inliers_ptr->size() == 0 || cor_inliers_ptr->size() == new_correspondences->size() || abs(transformation(0,3))>0.5 || abs(transformation(1,3))>0.5 || abs(transformation(2,3))>1 || transformation(0,0)<0.99){
        //Bruker icp hvis keypoint transformasjonen ikke er bra. Kan vaere bedre med ndt her, det vet jeg ikke om, men skal ikke brukes veldig ofte
        Eigen::Matrix4f guess; 
        guess << 1, 0, 0, 0.2, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setInputSource(source_cloud);
        icp.setInputTarget(target_cloud);
        icp.setMaximumIterations(50);
        icp.setMaxCorrespondenceDistance(10);
        icp.setRANSACOutlierRejectionThreshold(0.05);
        icp.setTransformationEpsilon (1e-9);
        pcl::PointCloud<pcl::PointXYZI> Final;
        icp.align(Final);
        Eigen::Matrix4f current_transformation;
        current_transformation = icp.getFinalTransformation();
        // Eigen::Matrix3f rotation;
        rotation << current_transformation(0,0), current_transformation(0,1), current_transformation(0,2),current_transformation(1,0), current_transformation(1,1), current_transformation(1,2),current_transformation(2,0), current_transformation(2,1), current_transformation(2,2);
        current_rotation = rotation;
        total_rotation = total_rotation * rotation;
        since_last_rotation = since_last_rotation * rotation; 
        // Eigen::Vector3f translation;
        translation << current_transformation(0,3), current_transformation(1,3), current_transformation(2,3);
        current_translation = translation;
        total_translation = total_translation + total_rotation*translation;
        since_last_translation = since_last_translation + since_last_rotation*translation;
        since_last_transformation = since_last_transformation * current_transformation;
        distance_since_last += sqrt(pow(current_transformation(0,3),2) + pow(current_transformation(1,3),2) + pow(current_transformation(2,3), 2));
        std::cout << ros::WallTime::now() << ": " << total_translation(0) << ", " << total_translation(1) << ", " << total_translation(2) << ", " << acos(total_rotation(0,0)) << ", 1" << std::endl;
      } else {
        //Hvis transformasjonen er bra saa bruker vi den fra keypoints/descriptors
        Eigen::Matrix4f current_transformation;
        current_transformation = transformation;
        // Eigen::Matrix3f rotation;
        rotation << current_transformation(0,0), current_transformation(0,1), current_transformation(0,2),current_transformation(1,0), current_transformation(1,1), current_transformation(1,2),current_transformation(2,0), current_transformation(2,1), current_transformation(2,2);
        current_rotation = rotation;
        total_rotation = total_rotation * rotation; 
        since_last_rotation = since_last_rotation * rotation; 
        // Eigen::Vector3f translation;
        translation << current_transformation(0,3), current_transformation(1,3), current_transformation(2,3);
        current_translation = translation;
        total_translation = total_translation + total_rotation*translation;
        since_last_translation = since_last_translation + since_last_rotation*translation;
        since_last_transformation = since_last_transformation * current_transformation;
        distance_since_last += sqrt(pow(current_transformation(0,3),2) + pow(current_transformation(1,3),2) + pow(current_transformation(2,3), 2));
        std::cout << ros::WallTime::now() << ": " << total_translation(0) << ", " << total_translation(1) << ", " << total_translation(2) << ", " << acos(total_rotation(0,0)) << ", 2" << std::endl;
      }
    }


    Eigen::Affine3d T_relative;
    T_relative.translation() = translation.cast <double> ();
    T_relative.linear() = rotation.cast <double> ();
    geometry_msgs::PoseStamped pose_relative_msg = toPoseStamped(stamp_prev_measurement, T_relative);

    pose_pub.publish(pose_relative_msg);


    //Vi onsker ikke bruke alle point clouds for aa sjekke loop closures, saa bruker bare hver 4.
    if (slam_counter % 4 == 1){
      pcl::PointCloud<pcl::SHOT352>::Ptr feat_cor_ (new pcl::PointCloud<pcl::SHOT352> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr source_cor_ (new pcl::PointCloud<pcl::PointXYZI> ());
      
      source_cor_->resize (cor_inliers_ptr->size ()); 
      feat_cor_->resize (cor_inliers_ptr->size ());
      
      for (size_t i = 0; i < cor_inliers_ptr->size (); ++i)
      {
          source_cor_->points[i] = source_keypoints->points[ (*cor_inliers_ptr)[i].index_query];
          feat_cor_->points[i] = source_cloud_keypoints->points[ (*cor_inliers_ptr)[i].index_query];
      }

      pcl::transformPointCloud (*source_cor_, *trans_keypoints, since_last_transformation);
      *loop_keypoints = *loop_keypoints + *trans_keypoints;
      *loop_cloud_keypoints = *loop_cloud_keypoints + *feat_cor_;
    }

    //Onsker ikke sende over keypoints til back-end oftere enn vi maa eller sjekke loop closures. Saa gjor det hver 12 gang
    if ((slam_counter%12==2) || true){ // TODO: edit true 
    if (send_message) {
      send_message = false;
      for (size_t i = 0; i < loop->size (); ++i)
      {
        std::ostringstream ss;
        ss << loop2->points[i].data_c[1];
        point_msg.header.frame_id = ss.str();
        point.x = loop->points[i].x;
        point.y = loop->points[i].y;
        point.z = loop->points[i].z;

        std::cout << loop2->points[i].data_c[1] << " Map: " << "x: " << loop2->points[i].x << ", y: " << loop2->points[i].y << ", z: " << loop2->points[i].z << std::endl;
        std::cout << "Source: " << "x: " << loop->points[i].x << ", y: " << loop->points[i].y << ", z: " << loop->points[i].z << std::endl;

        point_msg.point = point;

        pub8.publish(point_msg);
        usleep(30);
      }

      for (size_t i = 0; i< source_cor->size(); ++i)
      {
        std::ostringstream ss;
        ss << source_cor->points[i].data_c[1];
        point_msg.header.frame_id = ss.str();
        point.x = source_cor->points[i].x;
        point.y = source_cor->points[i].y;
        point.z = source_cor->points[i].z;

        point_msg.point = point;
        pub8.publish(point_msg);
        usleep(30);
      }

      for (size_t i = 0; i < loop->size (); ++i)
      {
          map_translation << loop->points[i].x, loop->points[i].y, loop->points[i].z;
          map_total_translation = total_translation + total_rotation*map_translation;

          loop->points[i].x = map_total_translation(0);
          loop->points[i].y = map_total_translation(1);
          loop->points[i].z = map_total_translation(2);
      }

      *loop_closure_map = *loop_closure_map + *loop2;
      *loop_closure_map2 = *loop_closure_map2 + *loop;

      pcl::PCLPointCloud2 pcl_pc_2;
      pcl::toPCLPointCloud2(*loop_closure_map, pcl_pc_2);
      pub10.publish(pcl_pc_2);

      pcl::PCLPointCloud2 pcl_pc_3;
      pcl::toPCLPointCloud2(*loop_closure_map2, pcl_pc_3);
      pub11.publish(pcl_pc_3);
    } 
      geometry_msgs::PoseStamped pose_msg;
      geometry_msgs::Pose pose; 
      std::ostringstream ss;
      ss << counter;
      pose_msg.header.frame_id = ss.str();
      pose_msg.header.stamp = ros::Time::now();
      
      pcl::transformPointCloud (*loop_keypoints, *loop_keypoints, since_last_transformation.inverse());
      
      pose.position.x = since_last_translation(0);
      pose.position.y = since_last_translation(1);
      pose.position.z = since_last_translation(2);

      //Gjor om til quaternions fordi det er det vi maa sende
      float trace = since_last_rotation(0,0) + since_last_rotation(1,1) + since_last_rotation(2,2); 
      if( trace > 0 ) {
        float s = 0.5f / sqrtf(trace+ 1.0f);
        pose.orientation.w = 0.25f / s;
        pose.orientation.x = ( since_last_rotation(2,1) - since_last_rotation(1,2) ) * s;
        pose.orientation.y = ( since_last_rotation(0,2) - since_last_rotation(2,0) ) * s;
        pose.orientation.z = ( since_last_rotation(1,0) - since_last_rotation(0,1) ) * s;
      } else {
        if ( since_last_rotation(0,0) > since_last_rotation(1,1) && since_last_rotation(0,0) > since_last_rotation(2,2) ) {
          float s = 2.0f * sqrtf( 1.0f + since_last_rotation(0,0) - since_last_rotation(1,1) - since_last_rotation(2,2));
          pose.orientation.w = (since_last_rotation(2,1) - since_last_rotation(1,2) ) / s;
          pose.orientation.x = 0.25f * s;
          pose.orientation.y = (since_last_rotation(0,1) + since_last_rotation(1,0) ) / s;
          pose.orientation.z = (since_last_rotation(0,2) + since_last_rotation(2,0) ) / s;
        } else if (since_last_rotation(1,1) > since_last_rotation(2,2)) {
          float s = 2.0f * sqrtf( 1.0f + since_last_rotation(1,1) - since_last_rotation(0,0) - since_last_rotation(2,2));
          pose.orientation.w = (since_last_rotation(0,2) - since_last_rotation(2,0) ) / s;
          pose.orientation.x = (since_last_rotation(0,1) + since_last_rotation(1,0) ) / s;
          pose.orientation.y = 0.25f * s;
          pose.orientation.z = (since_last_rotation(1,2) + since_last_rotation(2,1) ) / s;
        } else {
          float s = 2.0f * sqrtf( 1.0f + since_last_rotation(2,2) - since_last_rotation(0,0) - since_last_rotation(1,1) );
          pose.orientation.w = (since_last_rotation(1,0) - since_last_rotation(0,1) ) / s;
          pose.orientation.x = (since_last_rotation(0,2) + since_last_rotation(2,0) ) / s;
          pose.orientation.y = (since_last_rotation(1,2) + since_last_rotation(2,1) ) / s;
          pose.orientation.z = 0.25f * s;
        }
      }
      pose_msg.pose = pose;

      //Sender posen
      pub7.publish(pose_msg);
      counter++;

      since_last_rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
      since_last_translation << 0, 0, 0;
      since_last_transformation << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
      
      pcl::PointCloud<pcl::SHOT352>::Ptr feat_cor (new pcl::PointCloud<pcl::SHOT352> ());
      source_cor->resize (cor_inliers_ptr->size ()); 
      feat_cor->resize (cor_inliers_ptr->size ());

      
      bool zero_test = false;
      
      int iter = 0;


      for (size_t i = 0; i < cor_inliers_ptr->size (); ++i)
      {
        //Fjerner keypoints som ikke er bra nok
        if ((*cor_inliers_ptr)[i].distance > 0.06 || abs(source_keypoints->points[(*cor_inliers_ptr)[i].index_query].x)>30 || abs(source_keypoints->points[(*cor_inliers_ptr)[i].index_query].y)>30){
          source_cor->points.erase(source_cor->begin()+iter);
          feat_cor->points.erase(feat_cor->begin()+iter);
        } else {
          source_cor->points[iter] = source_keypoints->points[ (*cor_inliers_ptr)[i].index_query];
          feat_cor->points[iter] = source_cloud_keypoints->points[ (*cor_inliers_ptr)[i].index_query];
          iter++;
        }
      }

      point_msg.header.stamp = ros::Time::now();
      pcl::PointCloud<pcl::PointXYZI>::Ptr map_loop (new pcl::PointCloud<pcl::PointXYZI> ());
      map_loop->resize (source_cor->size ()); 

      //Gjor om keypoints til riktig reference frame
      for (size_t i = 0; i < source_cor->size (); ++i)
      {
          map_translation << source_cor->points[i].x, source_cor->points[i].y, source_cor->points[i].z;
          nav_trans = total_translation;
          nav_rot = total_rotation;

          map_total_translation = nav_trans + nav_rot * map_translation;

          map_loop->points[i].x = map_total_translation(0);
          map_loop->points[i].y = map_total_translation(1);
          map_loop->points[i].z = map_total_translation(2);
 
          map_loop->points[i].data_c[1] = counter2;
          source_cor->points[i].data_c[1] = counter2;
          counter2++;
      }

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      pcl::ExtractIndices<pcl::SHOT352> extract;
      std::vector<int> indices;

      //Finner keypoints rundt naaverende posisjon
      pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZI> ());
      range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZI> ("x", pcl::ComparisonOps::LT, total_translation(0)+40.0)));
      range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZI> ("x", pcl::ComparisonOps::GT, total_translation(0)-40.0)));
      range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::LT, total_translation(1)+40.0)));
      range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::GT, total_translation(1)-40.0)));
      // build the filter
      pcl::ConditionalRemoval<pcl::PointXYZI> condrem(true);
      condrem.setCondition(range_cond);
      condrem.setInputCloud(map);
      // apply filter
      condrem.filter(*filtered_loop_keypoints);
      // pcl::IndicesConstPtr inliers2 = condrem.getRemovedIndices();
      condrem.getRemovedIndices(*inliers);

      extract.setInputCloud(feature_map);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*filtered_loop_cloud_keypoints);

      //Finner correspondences for loop closure
      boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr_ (new pcl::Correspondences);
      boost::shared_ptr<pcl::Correspondences> new_correspondences_2 (new pcl::Correspondences);
      pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> corr_est_;
      corr_est_.setInputSource (loop_cloud_keypoints);
      corr_est_.setInputTarget (filtered_loop_cloud_keypoints);
      corr_est_.determineReciprocalCorrespondences (*new_correspondences_2, 0.35);

      //RANSAC paa de ogsaa
      pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> correspondence_rejector;
      correspondence_rejector.setInputSource(loop_keypoints);
      correspondence_rejector.setInputTarget(filtered_loop_keypoints);
      correspondence_rejector.setInlierThreshold(0.1);
      correspondence_rejector.setMaximumIterations(1000);
      correspondence_rejector.setInputCorrespondences(new_correspondences_2);
      correspondence_rejector.getCorrespondences(*cor_inliers_ptr_);
      
      if (cor_inliers_ptr_->size () == new_correspondences_2->size ()){
        loop->resize (0); 
        loop2->resize (0);
      } else {
        loop->resize (cor_inliers_ptr_->size ()); 
        loop2->resize (cor_inliers_ptr_->size ()); 
      }

      int matches[cor_inliers_ptr_->size()];
      for (size_t i = 0; i < cor_inliers_ptr_->size (); ++i)
      {
        matches[i] = (*cor_inliers_ptr_)[i].index_match;
      }

      std::vector<int> matches_sorted (matches, matches + cor_inliers_ptr_->size());
      std::sort(matches_sorted.begin(), matches_sorted.end());
      
      int k=0;
      for (size_t i = 0; i < cor_inliers_ptr_->size (); ++i)
      {
        if (cor_inliers_ptr_->size () == new_correspondences_2->size ()){
          break;
        }
        k = cor_inliers_ptr_->size () - 1 - i;
        loop->points[i] = loop_keypoints->points[ (*cor_inliers_ptr_)[k].index_query];
        loop2->points[i] = filtered_loop_keypoints->points[ (*cor_inliers_ptr_)[k].index_match];
      } 

      if (send_message){
        *later_loop_keypoints = *later_loop_keypoints + *map_loop;
        *later_loop_cloud_keypoints = *later_loop_cloud_keypoints + *feat_cor;
        map_counter++;
      }

      if (map_counter % 5 == 1){
        *feature_map = *feature_map + *later_loop_cloud_keypoints;
        *map = *map + *later_loop_keypoints;
        later_loop_cloud_keypoints->resize(0);
        later_loop_keypoints->resize(0);
      }

      map->header.frame_id = point_cloud_ptr->header.frame_id;
      map->header.stamp = point_cloud_ptr->header.stamp;

      filtered_loop_keypoints->header.frame_id = point_cloud_ptr->header.frame_id;
      filtered_loop_keypoints->header.stamp = point_cloud_ptr->header.stamp;

      loop->header.frame_id = point_cloud_ptr->header.frame_id;
      loop->header.stamp = point_cloud_ptr->header.stamp;

      loop_closure_map->header.frame_id = point_cloud_ptr->header.frame_id;
      loop_closure_map->header.stamp = point_cloud_ptr->header.stamp;

      loop_closure_map2->header.frame_id = point_cloud_ptr->header.frame_id;
      loop_closure_map2->header.stamp = point_cloud_ptr->header.stamp;

      pcl::PCLPointCloud2 pcl_pc_;
      pcl::toPCLPointCloud2(*filtered_loop_keypoints, pcl_pc_);
      pub9.publish(pcl_pc_);

      loop_keypoints->resize(0);
      loop_cloud_keypoints->resize(0);

    }  

    slam_counter++;

    target_cloud_keypoints = shotFeatures;
    target_keypoints = model_keypoints;
    target_cloud = point_cloud_ptr;
}

//Pre-processing av point clouds skjer her
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    stamp_prev_measurement = cloud_msg->header.stamp;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond (new pcl::ConditionOr<pcl::PointXYZI> ());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZI> ("x", pcl::ComparisonOps::LT, -1.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZI> ("x", pcl::ComparisonOps::GT, 0.5)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::LT, -1.1)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::GT, 1.1)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(temp_cloud);
    // apply filter
    condrem.filter(*cloud_filtered);

    pcl::PointCloud<pcl::PointXYZI>::Ptr second_cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    // build the filter
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(1);
    outrem.setMinNeighborsInRadius(3);
    // apply filter
    outrem.filter (*second_cloud_filtered);

    pcl::PointCloud<pcl::PointXYZI>::Ptr third_cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(second_cloud_filtered);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(*third_cloud_filtered);

    iss(third_cloud_filtered);
}


int
main (int argc, char** argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  // Initialize ROS
  ros::init (argc, argv, "lidar_odometry");
  ros::NodeHandle nh;
  ros::NodeHandle nh2;


  // Read initial parameter values
	std::string filename, dataset;
	nh.getParam("/dataset", dataset);
	std::string config_path = ros::package::getPath("lidar_odometry") + "/../../../config/" + dataset + "/backend/";
	boost::property_tree::ptree parameters = readConfigFromJsonFile( config_path + "parameters.json" );

  Eigen::Quaternionf q(parameters.get< float >("pose.orientation.w"), 
                       parameters.get< float >("pose.orientation.x"), 
                       parameters.get< float >("pose.orientation.y"), 
                       parameters.get< float >("pose.orientation.z"));
  
  Eigen::Vector3f t(parameters.get< float >("pose.translation.x"), 
                    parameters.get< float >("pose.translation.y"), 
                    parameters.get< float >("pose.translation.z"));

  //Disse er hardkodet saa man begynner med samme lokasjon paa GPS og odometri
  since_last_transformation << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  // total_rotation << cos(0.83), sin(0.83), 0, -sin(0.83), cos(0.83), 0, 0, 0, 1;
  // total_translation << -0.645953733353, 2.79517319837, 0;
  since_last_rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  since_last_translation << 0, 0, 0;

  total_rotation = q.normalized().toRotationMatrix();
  total_translation = t;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("lidar_topic", 1, cloud_cb);

  //Jeg har helt sikkert fjernet en del av disse pub da jeg slettet masse funksjoner, burde ikke ha mye aa si, men burde nok fjerne de det gjelder her ogsaa. Det finner du ut av, kommer forhåpentligvis til å kompilere uansett
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);

  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("NARF_keypoints", 1);

  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("SIFT_keypoints", 1);

  pub4 = nh.advertise<sensor_msgs::PointCloud2> ("ISS_keypoints", 1);

  pub5 = nh.advertise<sensor_msgs::PointCloud2> ("Matching_points", 1);

  pub6 = nh.advertise<sensor_msgs::PointCloud2> ("Harris_keypoints", 1);

  pub7 = nh.advertise<geometry_msgs::PoseStamped> ("Pose_test", 1);

  pub8 = nh.advertise<geometry_msgs::PointStamped> ("Point_test", 1);

  pub9 = nh.advertise<sensor_msgs::PointCloud2> ("Loop", 1);

  pub10 = nh.advertise<sensor_msgs::PointCloud2> ("Loop_closure", 1);

  pub11 = nh.advertise<sensor_msgs::PointCloud2> ("Loop_closure2", 1);

  pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("/frontend/lidar_odometry/pose_relative", 1);

  // Spin
  ros::spin ();
}