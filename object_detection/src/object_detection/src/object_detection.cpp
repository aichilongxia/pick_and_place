//http://pointclouds.org/documentation/tutorials/cluster_extraction.php
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//PCL相关
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/shared_ptr.hpp>

//ROS数据类型和PCL数据类型之间的转换
#include <pcl_conversions/pcl_conversions.h>

//定义数据类型
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

ros::Publisher pub;
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
void objectDetectionCB(const sensor_msgs::PointCloud2Ptr& pointCloudMSG)
{
    //将ROS的点云消息转换成PCL格式的数据类型
    PointCloudT::Ptr pointCloudPCL(new PointCloudT);
    PointCloudT::Ptr cloud_f(new PointCloudT);
    pcl::fromROSMsg(*pointCloudMSG, *pointCloudPCL);

    //detect objects from the original point cloud
    std::cout << "PointCloud before filtering has: "  
              << pointCloudPCL->points.size() << " data points" << std::endl;
    //创建进行下采样的对象，并设置参数
    pcl::VoxelGrid<PointT> voxel;
    PointCloudT::Ptr cloud_filtered(new PointCloudT);  //下采样后的点云数据
    voxel.setInputCloud(pointCloudPCL);
    voxel.setLeafSize(0.005f, 0.005f, 0.005f);
    voxel.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: "
              << cloud_filtered->points.size() << " data points" << std::endl;
    
    //创建进行平面分割的对象并设置所有的参数
    pcl::SACSegmentation<PointT> sac;
    pcl::PointIndices::Ptr inliners(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    PointCloudT::Ptr cloud_plane(new PointCloudT);
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(0.01);
    sac.setMaxIterations(200);
    ROS_INFO("Plane segment!");
    int i = 0;
    int nr_points = (int)cloud_filtered->points.size();
    ROS_INFO("Begin while loop");
    while(cloud_filtered->points.size() > 0.3 * nr_points)
    //for(int i = 0; i < 3; i++)
    {
        //从剩余的点云中分割出最大的平面
        ROS_INFO("Already in while loop");
        sac.setInputCloud(cloud_filtered);
        sac.segment(*inliners, *coefficients);
        if(inliners->indices.size() == 0)
        {
            std::cout << "Could not estimate a planner model for the given dataset."
                      << std::endl;
            return;
        }
        
        //从输入的点云中提取平面内点
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliners);
        extract.setNegative(false);
        //获取平面表面的点云
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component " 
                  << cloud_plane->points.size() << " data points" << std::endl;
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }
    std::cout << "PointCloud remaning: " << cloud_filtered->points.size() << " data points"
              << std::endl;

    ROS_INFO("Create Kd Tree");
    //创建kd树
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_filtered);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> cluster_extraction;
    cluster_extraction.setClusterTolerance(0.005);
    cluster_extraction.setMinClusterSize(100);
    cluster_extraction.setMaxClusterSize(2500);
    cluster_extraction.setSearchMethod(tree);
    cluster_extraction.setInputCloud(cloud_filtered);
    cluster_extraction.extract(cluster_indices);
    ROS_INFO("ooo");
    //利用聚类的索引值将点云提取出来
    pcl::ExtractIndices<PointT> indices_extraction;
    indices_extraction.setInputCloud(cloud_filtered);
    indices_extraction.setNegative(false);
    PointCloudT single_block;
    PointCloudT blocks;
    ROS_INFO("xxcx");
    ROS_INFO("cluster_indices.size: %d", cluster_indices.size());
    for(int i = 0; i < cluster_indices.size(); i++)
    {
        ROS_INFO("dddd");
        pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices);
        *indices_ptr = cluster_indices[i];
        indices_extraction.setIndices(indices_ptr);
        indices_extraction.filter(single_block);
        blocks += single_block;       
    }
    ROS_INFO("xxx");
    sensor_msgs::PointCloud2 blocksForPub;
    pcl::toROSMsg(blocks, blocksForPub);
    blocksForPub.header.frame_id = pointCloudMSG->header.frame_id;
    blocksForPub.header.stamp = ros::Time::now();
    pub.publish(blocksForPub);
    ROS_INFO("End callback");
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detection");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::PointCloud2>("object_detected", 1);
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 
                                       10, 
                                       objectDetectionCB);    
    ros::spin();
    return 0;
}
