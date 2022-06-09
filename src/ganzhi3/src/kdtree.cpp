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
#include <pcl/filters/crop_box.h>
#include <geometry_msgs/PoseStamped.h>
#include "ganzhi3/RsPointXYZIRT.h" //定义了速腾的点云结构

using namespace std;
/*
        功能：
                订阅二维码中心坐标
                发布大小邻域搜索点云
*/

#define width 0.60  //二维码板的宽度
#define err_x 0.1   //相机雷达x偏移
#define err_y 0.0   //相机雷达y偏移
#define err_z -0.12 //相机雷达z偏移

#define samll_box_x 1.5
#define samll_box_y 0.2
#define samll_box_z 0.2
#define big_box_x 2.0
#define big_box_y 0.8
#define big_box_z 0.5

static string base_link = "kd_link"; // kdtree搜索的坐标系

// 定义邻域搜索类
class kdtreepub
{

public:
        // 构造函数
        kdtreepub()
        {
                // 订阅方
                // 订阅相机发布的二维码点坐标
                camera_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 1, &kdtreepub::camera_sub_cb, this);
                // 订阅原始点云
                rslidar_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 1, &kdtreepub::rslidar_sub_cb, this);

                // 发布方
                // 发布小邻域搜索点云
                kdtree_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/output_kdtree", 1);
                // 发布大邻域搜索点云
                kdtreeplus_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/output_kdtreeplus", 1);
        }

private:
        ros::NodeHandle nh;
        ros::Subscriber rslidar_sub_;    //订阅原始点云	订阅方
        ros::Subscriber camera_sub_;           //订阅相机发布的二维码点坐标	订阅方
        ros::Publisher kdtree_pc_pub_;     //发布小邻域搜索点云 发布方
        ros::Publisher kdtreeplus_pc_pub_; //发布大邻域搜索点云 发布方
        pcl::PointCloud<RsPointXYZIRT> searchPoint_sub;
        pcl::PointCloud<RsPointXYZIRT> searchPointplus_sub;
        RsPointXYZIRT searchPoint; //邻域搜索圆心

        // 订阅相机发布的二维码中心
        void camera_sub_cb(const geometry_msgs::PoseStampedConstPtr &cloud_msg2)
        {
                searchPoint.x = (cloud_msg2->pose.position.z) + err_x;  // 相机的z方向是雷达的x方向
                searchPoint.y = -(cloud_msg2->pose.position.x) + err_y; //相机的-x方向是雷达的+y方向
                // searchPointplus.z = searchPoint.z = -(cloud_msg2->pose.position.y) + err_z;	//相机的-y方向是雷达的+z方向
                searchPoint.z = err_z; //相机的-y方向是雷达的+z方向

                cout << "相机识别二维码圆心：" << "(x = " << searchPoint.x << ", y = " << searchPoint.y << ", z = " << searchPoint.z << " )" << endl;
        }

        void rslidar_sub_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg1)
        {
                pcl::PointCloud<RsPointXYZIRT>::Ptr original_cloud(new pcl::PointCloud<RsPointXYZIRT>); //存放雷达原始点云
                pcl::fromROSMsg(*cloud_msg1, *original_cloud);  //把雷达原始点云从ROS格式转换成RsPointXYZIRT格式并存放到original_cloud里
                sensor_msgs::PointCloud2 cloud_pt;      //存放要发布的小邻域搜索点云
                sensor_msgs::PointCloud2 cloud_ptplus;  //存放要发布的大邻域搜索点云
                pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtree(new pcl::PointCloud<RsPointXYZIRT>);   //存储小邻域搜索后的点云
                pcl::CropBox<RsPointXYZIRT> box_filter; //滤波器对象
                box_filter.setMin(Eigen::Vector4f(searchPoint.x - samll_box_x, searchPoint.y - samll_box_y, err_z - samll_box_z, 1.0)); // Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，齐次坐标）
                box_filter.setMax(Eigen::Vector4f(searchPoint.x + samll_box_x, searchPoint.y + samll_box_y, err_z + samll_box_z, 1.0));
                box_filter.setNegative(false);            // false是将盒子内的点去除，默认为false
                box_filter.setInputCloud(original_cloud); //输入源
                box_filter.filter(*cloud_kdtree);         //保留！
                pcl::toROSMsg(*cloud_kdtree, cloud_pt);   //把小邻域搜索到的点云转换成ROS的发布格式
                cloud_pt.header.stamp = ros::Time::now();
                cloud_pt.header.frame_id = base_link; //设置发布点云的坐标系：要设置在输出点云存储的容器中

                pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtreeplus(new pcl::PointCloud<RsPointXYZIRT>);       //存储大邻域搜索后的点云
                pcl::CropBox<RsPointXYZIRT> boxplus_filter;     //滤波器对象
                boxplus_filter.setMin(Eigen::Vector4f(searchPoint.x - big_box_x, searchPoint.y - big_box_y, err_z - big_box_z, 1.0)); // Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，齐次坐标）
                boxplus_filter.setMax(Eigen::Vector4f(searchPoint.x + big_box_x, searchPoint.y + big_box_y, err_z + big_box_z, 1.0));
                boxplus_filter.setNegative(false);      // false是将盒子内的点去除，默认为false
                boxplus_filter.setInputCloud(original_cloud);   //输入源
                boxplus_filter.filter(*cloud_kdtreeplus);       //保留！
                pcl::toROSMsg(*cloud_kdtreeplus, cloud_ptplus); //把大邻域搜索到的点云转换成ROS的发布格式
                cloud_ptplus.header.stamp = ros::Time::now();
                cloud_ptplus.header.frame_id = base_link; //设置发布点云的坐标系：要设置在输出点云存储的容器中

                kdtree_pc_pub_.publish(cloud_pt);         //发布小邻域搜索到的点云
                kdtreeplus_pc_pub_.publish(cloud_ptplus); //发布大邻域搜索到的点云
                return;
        }
}; //以上是kdtree的类

int main(int argc, char **argv)
{
        ros::init(argc, argv, "kdtree");
        kdtreepub kdtree_pub;

        ros::Rate loop_rate(10); //循环发布频率为10HZ
        while (ros::ok())
        {
                ros::spinOnce();
                loop_rate.sleep();
        }
        return 0;
}