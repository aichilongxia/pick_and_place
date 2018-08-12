#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>

#include "object_detection/point_cloud_preprocess.h"

#include <assert.h>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
const double PI = 3.1415926;

class Object_localization
{
private:
    ros::NodeHandle nh_;
    tf::TransformListener transform_listener_;

    ros::Publisher objects_pub_;
    ros::Publisher target_base_pub;//物体在base1下的坐标描述
  
    ros::Publisher target_height_pub;//height_thing
  

    ros::Subscriber point_cloud_sub_;
    std::string base_frame_;

    double voxel_leaf_size_;        //downsample parameter

    double sac_distance_threshold_; //palne remove parameter
    double sac_max_iterations_;
    double sac_probability_;

    double passthrough_x_max_;      //passthrough filters parameters
    double passthrough_x_min_;
    double passthrough_y_max_;
    double passthrough_y_min_;
    double passthrough_z_max_;
    double passthrough_z_min_;
    double cluster_tolerance_;
    double min_cluster_size_;
    double max_cluster_size_;
    double height_;
public:
    Object_localization():nh_("~")
    {
    	ROS_INFO("Constructor!");
        nh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.005);
        nh_.param<double>("sac_distance_threshold", sac_distance_threshold_, 0.03);
        nh_.param<double>("sac_max_iterations",     sac_max_iterations_,     200);
        nh_.param<double>("sac_probability",        sac_probability_,        0.95);
        nh_.param<double>("passthrough_x_max", passthrough_x_max_, 1.0);
        nh_.param<double>("passthrough_x_min", passthrough_x_min_, 0.0);
        nh_.param<double>("passthrough_y_max", passthrough_y_max_, 0.25);
        nh_.param<double>("passthrough_y_min", passthrough_y_min_, -0.25);
        nh_.param<double>("passthrough_z_max", passthrough_z_max_, 0.5);
        nh_.param<double>("passthrough_z_min", passthrough_z_min_, 0.3);
      
        nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.015);
        nh_.param<double>("min_cluster_size", min_cluster_size_, 10);
        nh_.param<double>("max_cluster_size", max_cluster_size_, 2500);

        nh_.param<std::string>("base_frame", base_frame_, "/base_link");
       
        objects_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("objects_detected", 10);
        target_base_pub = nh_.advertise<geometry_msgs::Pose>("/pick_and_place",1);
      
        target_height_pub=nh_.advertise<std_msgs::Float64>("/height_get",1);
       
        point_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 10, &Object_localization::cloudCB, this);
      
    }

    void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
    	
       ros::Time curr = ros::Time::now();
        //首先转化数据格式
        PointCloudT::Ptr cloud_in_camera_link(new PointCloudT);
        PointCloudT::Ptr cloud_in_base_link(new PointCloudT);
        pcl::fromROSMsg(*cloud_msg, *cloud_in_camera_link);
        
        //然后将订阅到的点云转化到机器人基坐标系下
        tf::StampedTransform camera_base_transform;
        try{
        	transform_listener_.lookupTransform(base_frame_,cloud_msg->header.frame_id,  
        	                                ros::Time::now(), camera_base_transform);
        }
        catch (tf::TransformException& ex){
        	ROS_WARN("TF exception: %s", ex.what());
        	return;
        }

        try{
        	pcl_ros::transformPointCloud(*cloud_in_camera_link, *cloud_in_base_link,
        	                         tf::Transform(camera_base_transform));
        }
        catch (tf::TransformException& ex){
            ROS_WARN("TF exception: %s", ex.what());
        	return;	
        }

        pointCloudPreprocess point_cloud_preprocess;
		point_cloud_preprocess.setPreProcessedParam(voxel_leaf_size_,
                sac_distance_threshold_, sac_max_iterations_,sac_probability_,
                passthrough_x_max_, passthrough_x_min_, passthrough_y_max_,
        	    passthrough_y_min_, passthrough_z_max_, passthrough_z_min_
				);

        PointCloudT::Ptr plane(new PointCloudT);
        PointCloudT::Ptr objects(new PointCloudT);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    	point_cloud_preprocess.setInputCloud(cloud_in_base_link);
        point_cloud_preprocess.executePreprocess(plane, objects, coefficients);

        //将平面点云以外的点云发布出去
        sensor_msgs::PointCloud2 objectsForPub;
        pcl::toROSMsg(*objects, objectsForPub);
        objectsForPub.header.frame_id = base_frame_;
        objects_pub_.publish(objectsForPub);

        //确保平面与地面平行，并且高度大于0
        //assert(fabs(coefficients->values[0]) <= 0.01 && fabs(coefficients->values[1]) <= 0.01);
        //assert(coefficients->values[2] * coefficients->values[3] < 0);
        std::cout << "a = " << coefficients->values[0] << ";" 
                  << "b = " << coefficients->values[1] << ";"
                  << "c = " << coefficients->values[2] << ";" 
                  << "d = " << coefficients->values[3] << std::endl;
        //然后对保留下来的物体执行聚类分割
        height_ = -1 * coefficients->values[3] / coefficients->values[2];
        assert(height_ > 0);


        //创建kd树
        
       
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(objects);
    
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> cluster_extraction;
        cluster_extraction.setClusterTolerance(cluster_tolerance_);
        cluster_extraction.setMinClusterSize(min_cluster_size_);
        cluster_extraction.setMaxClusterSize(max_cluster_size_);
        cluster_extraction.setSearchMethod(tree);
        cluster_extraction.setInputCloud(objects);
        cluster_extraction.extract(cluster_indices);

        pcl::ExtractIndices<PointT> indices_extraction;
        indices_extraction.setInputCloud(objects);
        indices_extraction.setNegative(false);
      
        std::cout << cluster_indices.size() << "objects found" << std::endl;
        
        double mean_x = 0;
        double mean_y = 0;
        double height = 0;

        for(int i = 0; i < cluster_indices.size(); i++)
        {
            PointCloudT::Ptr single_object(new PointCloudT);
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices);
            *indices_ptr = cluster_indices[i];
            indices_extraction.setIndices(indices_ptr);
            indices_extraction.filter(*single_object);

            //compute_feature(single_object);
            //向桌面投影
            PointCloudT::Ptr cloud_projected(new PointCloudT);
            pcl::ProjectInliers<PointT> proj;
            proj.setModelType (pcl::SACMODEL_PLANE);
            proj.setInputCloud (single_object);
            proj.setModelCoefficients (coefficients);
            proj.filter (*cloud_projected);

            double sum_x = 0;
            double sum_y = 0;
           
            int size = cloud_projected->size();
            std::cout<<"size = "<<size<<std::endl;
            for(int i = 0; i < size; i++)
            {
                sum_x += cloud_projected->points[i].x;
                sum_y += cloud_projected->points[i].y;
            }

            mean_x = sum_x/size;
            mean_y = sum_y/size;

            double min_height = single_object->points[0].z;
            double max_height = single_object->points[0].z;

            for(int i = 0; i < single_object->size(); i++)
            {
            	min_height = std::min(min_height, (double)single_object->points[i].z);
            	max_height = std::max(max_height, (double)single_object->points[i].z);
            }
            std::cout<<"object's size = "<<single_object->size()<<std::endl;
            height = max_height - min_height;
            std_msgs::Float64 height_thing;
            height_thing.data = max_height - min_height;
            target_height_pub.publish(height_thing);
 
        }

        geometry_msgs::Pose pose;
        pose.position.x = mean_x ;
        pose.position.y = mean_y;
        pose.position.z = height_ + height / 2;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        target_base_pub.publish(pose);//物体在base中的描述
        std::cout<<"x = "<<mean_x<<std::endl;
        std::cout<<"y = "<<mean_y<<std::endl;
        std::cout<<"object's height = "<<height<<std::endl;
        std::cout<<"Table's height"<<height_<<std::endl;
    }
    /*void compute_feature(const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  PointT minPt, maxPt;
  getMinMax3D (*cloud, minPt, maxPt);
  float height = maxPt.z-minPt.z;
  float x,y,z;
  x=0.5*(maxPt.x+minPt.x);
  y=0.5*(maxPt.y+minPt.y);
  z=0.5*(maxPt.z+minPt.z);
  std::cout<<"The height is :"<<height<<std::endl;
  
  std::cout<<"The centriod is :"<<x<<" "<<y<<" "<<z<<std::endl;
  
}  */
    
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Object_localization");
	Object_localization localization;
	ros::spin();
	return 0;
}
