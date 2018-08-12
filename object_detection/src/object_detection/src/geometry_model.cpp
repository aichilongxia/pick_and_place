#include "object_detection/geometry_model.h"

PointCloudGeometryInfo::PointCloudGeometryInfo():cloud_source(new PointCloudT){}

void PointCloudGeometryInfo::setInputCloud(PointCloudT::Ptr cloud_in)
{
  copyPointCloud (*cloud_in, *cloud_source);
}

void PointCloudGeometryInfo::setTableHeight(double table_height)
{
  this->table_height = table_height;
}
void PointCloudGeometryInfo::print_result()
{
  std::cout << "****This is the geometry model info of the point cloud****" << std::endl;
	printf("**  centroid.x is: %f\n", result.x);
	printf("**  centroid.y is: %f\n", result.y);
	printf("**  direction  is: %f\n", result.theta);
	printf("**  height     is: %f\n", result.height);
	printf("**  width      is: %f\n", result.width);
  std::cout << "End show info..........." << std::endl;
}


struct geometry_model PointCloudGeometryInfo::get_geometry_model()
{
  const double PI = 3.1415926;
  double sum_x = 0;
  double sum_y = 0;
  double mean_x = 0;
  double mean_y = 0;

  int size = cloud_source->size();
  double min_height = cloud_source->points[0].z;
  double max_height = cloud_source->points[0].z;
  for(int i = 0; i < size; i++)
  {
    //assert(fabs(cloud_source->points[i].z - table_height) < 0.01);//如果距离桌面高度在1cm内，则认为该点合理，否则报错出局
    sum_x += cloud_source->points[i].x;
    sum_y += cloud_source->points[i].y;

    min_height = std::min(min_height, (double)cloud_source->points[i].z);
    max_height = std::max(max_height, (double)cloud_source->points[i].z);
  }
  
  result.x = sum_x / size;
  result.y = sum_y / size;
  result.height = max_height - min_height;

  Eigen::MatrixXd m(2, size);//???????????????????
  for(int i = 0; i < size; i++)
  {
    m(0, i) = cloud_source->points[i].x - result.x;
    m(1, i) = cloud_source->points[i].y - result.y;
  }
  Eigen::MatrixXd m1(2, 2);
  m1 = m * m.transpose();

  Eigen::EigenSolver<Eigen::MatrixXd> es(m1);
  Eigen::Matrix2d d = es.pseudoEigenvalueMatrix();
  Eigen::Matrix2d v = es.pseudoEigenvectors();

  //std::cout << m1 << std::endl;
  //std::cout << "Eigen value matrix is:" << std::endl << d << std::endl;
  //std::cout << "Eigen vector matrix is:" << std::endl << v << std::endl;

  int index = -1;
  if(d(0, 0) > d(1, 1))
    index = 0;
  else 
    index = 1;
  double a = v(index, 0);
  double b = v(index, 1); //主轴的方向向量是(a,b),则直线方程是bx+ay+c=0

  result.direction_x = a;
  result.direction_y = b;
  
  double c = -1 * (b * result.x + a * result.y);
  double theta = atan2(v(1, index), v(0, index));
  if(theta <= 0) theta += PI;
  double e = sqrt(a*a + b*b);

  PointT first_point = cloud_source->points[0];
  double max_width = (b * first_point.x + a * first_point.y + c) / e;
  double min_width = max_width;

  for(int i = 0; i < size; i++)
  {
  	PointT single_point = cloud_source->points[i];
  	double curr_width = (b * single_point.x + a * single_point.y + c) / e;
  	max_width = std::max(max_width, curr_width);
  	min_width = std::min(min_width, curr_width);
  }
  result.width = max_width - min_width;
  result.theta = theta;

  return result;   
}
