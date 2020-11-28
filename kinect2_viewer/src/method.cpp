#include "method.h"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>



using namespace std;


int* random(int size) {
    int *num = new int[1024];
    int i, j;
    srand(time(NULL));
    if (size > 1024){
    	for (i = 0; i < 1024;) {
        	num[i] = rand() % size;
        	for (j = 0; j < i; j++) {
           	if (num[j] == num[i]) break;
        	}
        	if (j == i) i++;
    	}
    }
    else{
    	for (i = 0; i < 1024;) {
    		num[i] = rand()%1024;
    		i++;
    	}
    }
    return num;
  }


void down_sampling(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int num){

    pcl::RandomSample<pcl::PointXYZ> rs;
    rs.setInputCloud(cloud);
    rs.setSample(num);
    rs.filter(*cloud);
}


void st_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Static;   //创建滤波器对象
	Static.setInputCloud(cloud);                           //设置待滤波的点云
	Static.setMeanK(18);                               //设置在进行统计时考虑查询点临近点数
	Static.setStddevMulThresh(0.08);                      //设置判断是否为离群点的阀值
	Static.filter(*cloud);

}


void down_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float y_limit, double d){

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
	pass_y.setFilterFieldName("y");
	//z轴区间设置
	pass_y.setFilterLimits(y_limit-d, y_limit);
	//pass_y.setFilterLimitsNegative(false);
	pass_y.filter(*cloud);

}

void back_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float z_limit, double d){

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
	pass_z.setFilterFieldName("z");
	//z轴区间设置
	pass_z.setFilterLimits(z_limit, z_limit+d);
	pass_z.setFilterLimitsNegative(false);
	pass_z.filter(*cloud);
}

void use_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &a){
	pcl::PointXYZ min;
    pcl::PointXYZ max;
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud, *a, mapping);

	pcl::getMinMax3D(*a,min,max);
	back_filter(a, min.z, 0.24);  
	int size  = a->width * a->height;
	if (size != 0){

		st_filter(a);
	}
	
	pcl::getMinMax3D(*a,min,max);
	back_filter(a, min.z, 0.13);  

	pcl::getMinMax3D(*a,min,max);
	down_filter(a, max.y, 0.19);
}