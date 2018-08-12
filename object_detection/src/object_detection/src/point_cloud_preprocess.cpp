#include <object_detection/point_cloud_preprocess.h>

pointCloudPreprocess::pointCloudPreprocess():cloud_processed_(new PointCloudT),
                                             cloud_original_(new PointCloudT),
                                             inliers_(new pcl::PointIndices),
                                             coefficients_(new pcl::ModelCoefficients)
{
	param_setted_ = false;
}

void pointCloudPreprocess::executePreprocess(PointCloudT::Ptr plane, PointCloudT::Ptr objects, 
                       pcl::ModelCoefficients::Ptr coefficients)
{
	assert(param_setted_);
    //点云下采样的具体执行
    std::cout << "downsample" << std::endl;
    downsample_.setInputCloud(cloud_original_);
    downsample_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_,
         	                    voxel_leaf_size_);
    downsample_.filter(*cloud_processed_);
    std::cout << "PointCloud after filtering has: "
              << cloud_processed_->points.size() << " data points" << std::endl;
    //分别执行x，y，z三个方向的直通滤波
    std::cout << "pass_through filter" << std::endl;
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    pass_through_.setInputCloud(cloud_processed_);
    pass_through_.setFilterFieldName("x");
    pass_through_.setFilterLimits(passthrough_x_min_, passthrough_x_max_);
    pass_through_.filter(*cloud_filtered);
    //cloud_processed_ = cloud_filtered;
    copyPointCloud (*cloud_filtered, *cloud_processed_); 

    pass_through_.setInputCloud(cloud_processed_);
    pass_through_.setFilterFieldName("y");
    pass_through_.setFilterLimits(passthrough_y_min_, passthrough_y_max_);
    cloud_filtered->points.resize(0);
    pass_through_.filter(*cloud_filtered);
    //cloud_processed_ = cloud_filtered;
    copyPointCloud (*cloud_filtered, *cloud_processed_);

    pass_through_.setInputCloud(cloud_processed_);
    pass_through_.setFilterFieldName("z");
    pass_through_.setFilterLimits(passthrough_z_min_, passthrough_z_max_);
    cloud_filtered->points.resize(0);
    pass_through_.filter(*cloud_filtered);
    //cloud_processed_ = cloud_filtered;
    copyPointCloud (*cloud_filtered, *cloud_processed_);
    //copyPointCloud(*cloud_filtered, pointCloud);
    //作业平面滤除
    std::cout << "plane_remove" << std::endl;
    plane_remove_.setInputCloud(cloud_processed_);
    plane_remove_.setMethodType(pcl::SAC_RANSAC);
    plane_remove_.setModelType(pcl::SACMODEL_PLANE);
    plane_remove_.setDistanceThreshold(sac_distance_threshold_);
    plane_remove_.setMaxIterations(sac_max_iterations_);
    plane_remove_.setProbability(sac_probability_);
    std::cout << "begin segment" << std::endl;
    plane_remove_.segment(*inliers_, *coefficients);
    std::cout << "end segment" << std::endl;
    //cloud_filtered->points.resize(0);
    ei_.setIndices(inliers_);
    ei_.setInputCloud(cloud_processed_);
    ei_.setNegative(true);
    ei_.filter(*cloud_filtered);
    //cloud_processed_ = cloud_filtered;
    copyPointCloud(*cloud_filtered, *objects);

    ei_.setNegative(false);
    ei_.filter(*cloud_filtered);
    copyPointCloud(*cloud_filtered, *plane);
    std::cout << "End executePreprocess" << std::endl;
    std::cout << " plane PointCloud remaning: " << plane->points.size() << " data points"
              << std::endl;
    std::cout << " object PointCloud remaning: " << objects->points.size() << " data points"
              << std::endl;
}

void pointCloudPreprocess::setInputCloud(PointCloudT::Ptr cloud_in)
{
	std::cout << "Begin setInputCloud" << std::endl;
	//cloud_original_ = cloud_in;
	copyPointCloud (*cloud_in, *cloud_original_);
	std::cout << "End setInputCloud" << std::endl;
}

void pointCloudPreprocess::setPreProcessedParam(double voxel_leaf_size,
     double sac_distance_threshold, double sac_max_iterations,double sac_probability,
     double passthrough_x_max, double passthrough_x_min, double passthrough_y_max,
     double passthrough_y_min, double passthrough_z_max, double passthrough_z_min)
{
	voxel_leaf_size_        = voxel_leaf_size;

	sac_distance_threshold_ = sac_distance_threshold;
	sac_max_iterations_     = sac_max_iterations;
	sac_probability_        = sac_probability;
 
        passthrough_x_max_      = passthrough_x_max;
        passthrough_x_min_      = passthrough_x_min;
        passthrough_y_max_      = passthrough_y_max;
	passthrough_y_min_      = passthrough_y_min;
	passthrough_z_max_      = passthrough_z_max;
	passthrough_z_min_      = passthrough_z_min;

	param_setted_ = true;
}
