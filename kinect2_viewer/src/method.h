#ifndef METHOD_H
#define METHOD_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void st_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
void down_sampling(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int num);
void use_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &a);
int* random(int size);

void down_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float y_limit, double d);

void back_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float z_limit, double d);

#endif