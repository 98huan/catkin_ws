#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include "ganzhi3/RsPointXYZIRT.h"		//定义了速腾的点云结构

using namespace std;
/*
	功能：
		订阅二维码中心坐标
		发布大小邻域搜索点云
*/

static float radius = 0.2;				//小邻域搜索半径
static float radiusplus = 0.6;			//大邻域搜索半径
static string base_link = "kd_link";	//kdtree搜索的坐标系

// 定义邻域搜索类
class kdtreepub{

public:
	// 构造函数
	kdtreepub(){
		// 订阅方
		// 订阅相机发布的二维码点坐标	
  		kp_sub_ = nh.subscribe<geometry_msgs::PoseStamped> ("/aruco_single/pose", 1, &kdtreepub::cloud_cb2, this);
		// 订阅原始点云
  		source_pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2> ("/rslidar_points", 1, &kdtreepub::cloud_cb1, this);

		// 发布方
		// 发布小邻域搜索点云
		kdtree_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/output_kdtree", 1);
		// 发布大邻域搜索点云
		kdtreeplus_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/output_kdtreeplus", 1);
	}
private:
	ros::NodeHandle nh;
  	ros::Subscriber source_pc_sub_;			//订阅原始点云	订阅方
  	ros::Subscriber kp_sub_;						//订阅相机发布的二维码点坐标	订阅方
  	ros::Publisher kdtree_pc_pub_;			//发布小邻域搜索点云 发布方
  	ros::Publisher kdtreeplus_pc_pub_;	//发布大邻域搜索点云 发布方
	pcl::PointCloud<RsPointXYZIRT> searchPoint_sub;
	pcl::PointCloud<RsPointXYZIRT> searchPointplus_sub;
	RsPointXYZIRT  searchPoint;						//大邻域搜索圆心
	RsPointXYZIRT  searchPointplus;				//小邻域搜索圆心

// 订阅相机发布的二维码中心
void cloud_cb2(const geometry_msgs::PoseStampedConstPtr& cloud_msg2)
{
	searchPointplus.x = searchPoint.x = (cloud_msg2->pose.position.z) + 0.040;			// 相机的z方向是雷达的x方向
	searchPointplus.y = searchPoint.y = -(cloud_msg2->pose.position.x) + 0.071;			//相机的-x方向是雷达的+y方向
	searchPointplus.z = searchPoint.z = -(cloud_msg2->pose.position.y) - 0.12;				//相机的-y方向是雷达的+z方向
	cout << "相机识别二维码圆心：" << "(x = " << cloud_msg2->pose.position.z << ", y = " <<  -(cloud_msg2->pose.position.x) << ", z = " <<  searchPoint.z << " )" << endl;
	
}

void cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg1)
{
	pcl::PointCloud<RsPointXYZIRT>::Ptr original_cloud (new pcl::PointCloud<RsPointXYZIRT>);	//存放雷达原始点云
    pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtree(new pcl::PointCloud<RsPointXYZIRT>);		//存储小邻域搜索后的点云
    pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtreeplus(new pcl::PointCloud<RsPointXYZIRT>);	//存储大邻域搜索后的点云
	sensor_msgs::PointCloud2 cloud_pt;					//存放要发布的小小小小邻域搜索点云
	sensor_msgs::PointCloud2 cloud_ptplus;			//存放要发布的大大大大邻域搜索点云
	pcl::fromROSMsg(*cloud_msg1, *original_cloud);	//把雷达原始点云从ROS格式转换成RsPointXYZIRT格式并存放到original_cloud里
	
	//创建kdtree 结构
	pcl::KdTreeFLANN<RsPointXYZIRT> kdtree;
	pcl::KdTreeFLANN<RsPointXYZIRT> kdtreeplus;

    //传入原始点云用于邻域搜索
    kdtree.setInputCloud(original_cloud);
    kdtreeplus.setInputCloud(original_cloud);

	// 创建两个向量，分别存放近邻的索引值、近邻的中心距
	std::vector<int> pointIdxRadiusSearch;										//索引值
	std::vector<float> pointRadiusSquaredDistance;					//中心距
	std::vector<int> pointIdxRadiusSearch_plus;							//索引值
	std::vector<float> pointRadiusSquaredDistance_plus;		//中心距
	
	cloud_kdtree->header.frame_id = base_link;				//设置发布点云的坐标系：要设置在输出点云存储的容器中
	cloud_kdtreeplus->header.frame_id = base_link;		//设置发布点云的坐标系：要设置在输出点云存储的容器中

	// 搜索到的点数 =  邻域搜索
	int num1  = kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	int num2  = kdtreeplus.radiusSearch (searchPointplus, radiusplus, pointIdxRadiusSearch_plus, pointRadiusSquaredDistance_plus);

	// 把小邻域搜索到的点放进容器中
	for (size_t i = 0; i < num1; ++i)
	{
	  	cloud_kdtree->push_back(original_cloud->points[pointIdxRadiusSearch[i]]);
	}
	// 把大邻域搜索到的点放进容器中
	for (size_t i = 0; i < num2; ++i)
	{
	  	cloud_kdtreeplus->push_back(original_cloud->points[pointIdxRadiusSearch_plus[i]]);
	}

	pcl::toROSMsg(*cloud_kdtree, cloud_pt);						  //把小邻域搜索到的点云转换成ROS的发布格式
	pcl::toROSMsg(*cloud_kdtreeplus, cloud_ptplus);		//把大邻域搜索到的点云转换成ROS的发布格式
    kdtree_pc_pub_.publish(cloud_pt);					    	  	   //发布小邻域搜索到的点云
    kdtreeplus_pc_pub_.publish(cloud_ptplus);				 //发布大邻域搜索到的点云
	return;
}
}; //以上是kdtree的类

int
main (int argc, char** argv)
{
  	// Initialize ROS
  	ros::init (argc, argv, "kdtree");
	kdtreepub kdtree_pub;

    ros::Rate loop_rate(10);        //循环发布频率为10HZ
  	while (ros::ok()) {
	    ros::spinOnce();
    	loop_rate.sleep();
	}
}
