#ifndef _GEOMETRY_MODEL_H_
#define _GEOMETRY_MODEL_H_

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues>

#include <pcl/io/pcd_io.h>  //读取pcd文件
#include <pcl/point_cloud.h>//定义基本的数据类型

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct geometry_model
{
  double x;
  double y;
  double height;
  double theta;	
  double width;
  //line model ax+by+c = 0
  double direction_x;
  double direction_y;
};

class PointCloudGeometryInfo
{
public:
  PointCloudGeometryInfo();
  struct geometry_model get_geometry_model();
  void setInputCloud(PointCloudT::Ptr cloud_in);
  void setTableHeight(double);
  void print_result();
private:
  PointCloudT::Ptr cloud_source;
  struct geometry_model result;
  double table_height;

};

#endif