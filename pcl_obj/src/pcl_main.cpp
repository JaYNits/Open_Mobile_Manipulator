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
        pcl::PointCloud<PointT> cloud;
        pcl::PointCloud<pcl::Normal> cloud_normals;
        pcl::PCDWriter writer;
  //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<PointT> cloud_segmented;
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud);
        // outlier removal
        pcl::StatisticalOutlierRemoval<PointT> statFilter;
        statFilter.setInputCloud(cloud.makeShared());
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(cloud);
        // downsampling 
        pcl::VoxelGrid<PointT> voxelSampler;
        voxelSampler.setInputCloud(cloud.makeShared());
        voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelSampler.filter(cloud);
        // plane segmentation 
        
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        //create segmentation object 
        pcl::SACSegmentation<PointT> segmentation;
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(1000);
        segmentation.setDistanceThreshold(0.01);
        segmentation.setInputCloud(cloud.makeShared());
        segmentation.segment(*inliers, coefficients);

        // Create the filtering object
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(cloud);



        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud.makeShared());

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud.makeShared());
        ec.extract (cluster_indices);
        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                cloud_cluster->points.push_back (cloud.points[*pit]); //*

            }
            pcl::PointXYZ avg;
            for(size_t i = 0; i < cloud_cluster->points.size(); i++){
                if(!isnan(cloud_cluster->points[i].x) && !isnan(cloud_cluster->points[i].y) && !isnan(cloud_cluster->points[i].z)) {
                    avg.x += cloud_cluster->points[i].x;
                    avg.y += cloud_cluster->points[i].y;
                    avg.z += cloud_cluster->points[i].z;
                }
            }
            avg.x /= cloud_cluster->points.size();
            avg.y /= cloud_cluster->points.size();
            avg.z /= cloud_cluster->points.size();
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            if(cloud_cluster->width > 200 && cloud_cluster->width < 220)
            {
                static tf::TransformBroadcaster br;
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(avg.x,avg.y,avg.z) );
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "object"));
                break;
            }
            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            std::stringstream ss;
            ss << "cloud_cluster_" << j << ".pcd";
            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
            j++;
        }
        // Create cylinder object
        /*segmentation.setOptimizeCoefficients (true);
        segmentation.setModelType(pcl::SACMODEL_CYLINDER);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setNormalDistanceWeight (0.1);
        segmentation.setMaxIterations (10000);
        segmentation.setDistanceThreshold (0.05);
        segmentation.setRadiusLimits (0, 0.1);
        segmentation.setInputCloud(cloud.makeShared());
        segmentation.setInputNormals(cloud_normals2.makeShared());
        segmentation.segment(*inliers, coefficients);


        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(cloud_normals);*/

        pcl::toROSMsg(cloud, output);
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

