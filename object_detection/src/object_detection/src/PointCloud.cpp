//ros header files
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>

#include <cmath>
#include <stdlib.h>
//pcl header file
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

std::string people_detection_frame_id = "people_detection_frame";

ros::Publisher pub;
boost::mutex cloud_mutex;


//pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//point_clouds_ros: point clouds in ros data type; need to be converted to pcl data type
void cloud_cb(const sensor_msgs::PointCloud2::Ptr& point_clouds_ros)
{
    ROS_INFO("Begin the call_back function!");
    
    //tf::TransformBroadcaster br;
    //tf::Transform detectionFrameToLaser(tf::Quaternion(0.500, 0.500, 0.500, 0.500), tf::Vector3(0.113, 0.012, -0.13));

    //br.sendTransform(tf::StampedTransform(detectionFrameToLaser, ros::Time::now(),
                                         //"base_laser_link", people_detection_frame_id));
    std::string svm_filename = "/home/sunzifei/svm.yaml";
    double min_confidence = -1.500;
    double min_height     = 1.3;
    double max_height     = 2.3;
    double voxel_size     = 0.01;
    //double error          = 0.05;

    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

    cloud_mutex.lock ();
    //Container for original & filtered data
    pcl::PCLPointCloud2* cloud_pcl = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_pcl);

    //convert to pcl data type:Pointcloud2
    pcl_conversions::toPCL(*point_clouds_ros, *cloud_pcl);
    PointCloudT* pointcloud_xyzrgb_orig = new PointCloudT;
    PointCloudT::Ptr pointcloud_xyzrgb(pointcloud_xyzrgb_orig);

    //convert from PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZRGBA>
    fromPCLPointCloud2(*cloud_pcl, *pointcloud_xyzrgb_orig);

    Eigen::VectorXf ground_coeffs;
    ground_coeffs.resize(4);

    //The grouud_coeffs,which is depended by specific scene, is inputed by user
    //And it can be resided in launch file such that when the parameter is changed,
    //there is no need to recompile the program.
    ground_coeffs(0) = 0.000761751;            
    ground_coeffs(1) = -0.998911;
    ground_coeffs(2) = -0.046658;
    ground_coeffs(3) = 0.508452;          

    pcl::people::PersonClassifier<pcl::RGB> person_classifier;
    person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

    // People detection app initialization:
    pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object

    people_detector.setVoxelSize(voxel_size);                              // set the voxel size
    people_detector.setIntrinsics(rgb_intrinsics_matrix);                  // set RGB camera intrinsic parameters
    people_detector.setClassifier(person_classifier);                      // set person classifier
    //people_detector.setHeightLimits(min_height, max_height);               // set person classifier
 
    std::vector<pcl::people::PersonCluster<PointT> > clusters;//(10, cluster);   // vector containing persons clusters
    people_detector.setInputCloud(pointcloud_xyzrgb);            
    people_detector.setGround(ground_coeffs);                    // set floor coefficients
    
    //Perform people detection on the input data and return people clusters information.
    //true if the compute operation is successful, false otherwise.
    bool flag = people_detector.compute(clusters);                           

    std::cout << "flag( people_detector.compute ) = " << flag << std::endl;

    pcl::PointIndices* indices = new pcl::PointIndices;
    pcl::PointIndices::Ptr people_indices(indices);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(people_detector.getNoGroundCloud());
    extract.setNegative(false);

    PointCloudT pointcloud_people;
    PointCloudT single_people;

    unsigned int k = 0;
    for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        
        if(it->getPersonConfidence() > min_confidence)            
        {
            k++;
            *people_indices = it->getIndices();

            extract.setIndices(people_indices);
            extract.filter(single_people);

            for(PointCloudT::iterator iter =  single_people.begin(); iter != single_people.end(); ++iter)
            {
              iter->rgb = k;
              pointcloud_people.push_back(*iter);
            }        
        }
    }

    PointCloudT cloud_projected;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    coefficients->values.resize (4);
    coefficients->values[0] = ground_coeffs(0);//-0.0673329
    coefficients->values[1] = ground_coeffs(1);//-0.802218
    coefficients->values[2] = ground_coeffs(2);//-0.593222
    coefficients->values[3] = ground_coeffs(3);

    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(pointcloud_people.makeShared());
    proj.setModelCoefficients(coefficients);
    proj.filter(cloud_projected);
/*
    FILE* people = fopen("/home/sunzifei/people.txt", "a");
    if(!people)
    {
      ROS_ERROR("Failed to open people.txt!");
      return;
    }

    for(int i = 0; i < pointcloud_people.size(); i++)
      fprintf(people, "%lf  %lf  %lf\n", pointcloud_people.points[i].x, pointcloud_people.points[i].y,
                                         pointcloud_people.points[i].z);
    fclose(people);
*/
    pcl::PCLPointCloud2 pointcloud2_people;
    sensor_msgs::PointCloud2 people_pointcloud_ros, people_pointcloud_ros_transform;

    toPCLPointCloud2(cloud_projected, pointcloud2_people);
    pcl_conversions::fromPCL(pointcloud2_people, people_pointcloud_ros);

    /*
     * When the people has been detected and the point cloud data about people extracted, we will change 
     * the frame_id and broadcast the transform between the current frame and laser frame to tf, with people 
     * detect frame child, and laser parent. 
     */
    //static tf::TransformBroadcaster br;
    //tf::Transform detectionFrameToLaser(tf::Quaternion(0.500, 0.500, 0.500, 0.500), tf::Vector3(0.113, 0.012, -0.13));
    //br.sendTransform(tf::StampedTransform(detectionFrameToLaser, ros::Time::now(),
                                          ///"base_laser_link", people_detection_frame_id));
    people_pointcloud_ros.header.frame_id = point_clouds_ros->header.frame_id;
    people_pointcloud_ros.header.seq   = point_clouds_ros->header.seq;
    people_pointcloud_ros.header.stamp = point_clouds_ros->header.stamp;

    //std::string target_frame = "base_laser_link";  //This param can remain in launch file for users to input

    //tf::StampedTransform peopleToBaseLink_Stamped;
    //tf::TransformListener listener(ros::Duration(10));
    //listener.lookupTransform(target_frame, people_detection_frame_id, ros::Time(0), peopleToBaseLink_Stamped);

    //listener.transformPointCloud(target_frame, people_pointcloud_ros, people_pointcloud_ros_transform);
    //Eigen::Affine3d peopleToBaseLinkEigen;
    //tf::Transform peopleToBaseLink(peopleToBaseLink_Stamped.getBasis(), peopleToBaseLink_Stamped.getOrigin());
    //tf::transformTFToEigen(peopleToBaseLink, peopleToBaseLinkEigen);

    //PointCloudT people_pointcloud_pcl;
    //PointCloudT people_in_laser_link;
    //pcl::fromROSMsg(*point_cloud_people, people_pointcloud_pcl);
    //pcl::transformPointCloud(cloud_projected, people_in_laser_link, peopleToBaseLinkEigen);

    std::cout << k << " people found" << std::endl;
    
    pub.publish(people_pointcloud_ros);
    cloud_mutex.unlock(); 

    ROS_INFO("Has ended call_back function!");
    return;

}


int main(int argc, char** argv)
{
    //Initialize ROS
    ros::init(argc, argv, "PointCloud");
    ros::NodeHandle nh;
    ROS_INFO("Has begin!");
    

    //Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 10, cloud_cb);
    ROS_INFO("Has sub.");
    pub = nh.advertise<sensor_msgs::PointCloud2>("people_detected",1);

    ROS_INFO("Has ended.");
    ros::spin();
    return 0;
}
