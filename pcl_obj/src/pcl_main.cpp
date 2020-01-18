#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZ PointT;

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("/camera/depth_registered/points", 10, &cloudHandler::cloudCB, this);
        //pcl_obj_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_obj", 1);
        pcl_plane_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_plane", 1);
        //ind_pub = nh.advertise<pcl_msgs::PointIndices>("point_indices", 1);
        //coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {


	// objects
  pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	sensor_msgs::PointCloud2 output;
	// datasets
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  	pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

	  //read data to pcl
    pcl::fromROSMsg(input, *cloud);
        //std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

        // Build a passthrough filter to remove spurious NaNs
  	//pass.setInputCloud (cloud);
  	//pass.setFilterFieldName ("z");
  	//pass.setFilterLimits (0, 1.5);
  	//pass.filter (*cloud_filtered);
  	//std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  //Downsampling
  voxelSampler.setInputCloud(cloud);
  voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
  voxelSampler.filter(*cloud);

 	// Estimate point normals
  	ne.setSearchMethod (tree);
  	ne.setInputCloud (cloud);//_filtered);
  	ne.setKSearch (80);//50
  	ne.compute (*cloud_normals);

  	// Create the segmentation object for the planar model and set all the parameters
  	seg.setOptimizeCoefficients (true);
  	seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  	seg.setNormalDistanceWeight (0.1);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (100);
  	seg.setDistanceThreshold (0.03);
  	seg.setInputCloud (cloud);//_filtered);
  	seg.setInputNormals (cloud_normals);
  	// Obtain the plane inliers and coefficients
  	seg.segment (*inliers_plane, *coefficients_plane);
  	//std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  	// Extract the planar inliers from the input cloud
  	extract.setInputCloud (cloud);//_filtered);
  	extract.setIndices (inliers_plane);
  	extract.setNegative (false);

  	// Write the planar inliers to disk
  	pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  	extract.filter (*cloud_plane);
  	//std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  	//writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  	// Remove the planar inliers, extract the rest
  	extract.setNegative (true);
  	extract.filter (*cloud_filtered2);
  	extract_normals.setNegative (true);
  	extract_normals.setInputCloud (cloud_normals);
  	extract_normals.setIndices (inliers_plane);
  	extract_normals.filter (*cloud_normals2);

  	// Create the segmentation object for cylinder segmentation and set all the parameters
  	seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
  	//seg.setModelType (pcl::SACMODEL_CYLINDER);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setNormalDistanceWeight (0.1);
  	seg.setMaxIterations (10000);
  	seg.setDistanceThreshold (0.05);
  	seg.setRadiusLimits (0, 0.07);
  	seg.setInputCloud (cloud_filtered2);
  	seg.setInputNormals (cloud_normals2);

  	// Obtain the cylinder inliers and coefficients
  	seg.segment (*inliers_cylinder, *coefficients_cylinder);
  	//std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  	// Write the cylinder inliers to disk
  	extract.setInputCloud (cloud_filtered2);
  	extract.setIndices (inliers_cylinder);
  	extract.setNegative (false);
  	pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  	extract.filter (*cloud_cylinder);
  	if (cloud_cylinder->points.empty ())
    		std::cerr << "Can't find the cylindrical component." << std::endl;
  	else
  	{
	  	std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
	  	//writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
  	}








        pcl::toROSMsg(*cloud_cylinder, output);
        pcl_plane_pub.publish(output);

    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_plane_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_main");

    cloudHandler handler;

    ros::spin();

    return 0;
}
