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
#include "/home/zh/catkin_ws/src/ganzhi3/include/ganzhi3/globle_center.h"
#include "/home/zh/catkin_ws/src/ganzhi3/include/ganzhi3/eul2quat.h"
#include "ganzhi3/RsPointXYZIRT.h" //定义了速腾的点云结构
using namespace std;
/*
        功能：
                订阅二维码中心坐标
                发布大小邻域搜索点云
*/

#define width 0.60	//二维码板的宽度
#define err_x 0.15	//相机雷达x偏移
#define err_y 0.071	//相机雷达y偏移
#define err_z -0.12	//相机雷达z偏移

#define samll_box_x 1.5
#define samll_box_y 0.25
#define samll_box_z 0.25

#define big_box_x 2.0
#define big_box_y 0.8
#define big_box_z 0.5

// 定义全局变量
extern float center_cam_x;
extern float center_cam_y;
extern float center_cam_z;

extern float center_lidar_x;
extern float center_lidar_y;
extern float center_lidar_z;

extern RsPointXYZIRT searchPoint;
static float radius = 0.4;           //小邻域搜索半径
static float radiusplus = 1.0;       //大邻域搜索半径
static string base_link = "kd_link"; // kdtree搜索的坐标系

// 定义邻域搜索类
class kdtreepub
{
        public:
                kdtreepub()     // 构造函数
                {
                        // 订阅方
                        // 订阅相机发布的二维码点坐标
                        camera_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 10, &kdtreepub::cloud_cb2, this);
                        // 订阅原始点云
                        lidar_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, &kdtreepub::cloud_cb1, this);
                        
                        // 发布方
                        // 发布小邻域搜索点云
                        kdtree_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/output_kdtree", 10);
                        // 发布大邻域搜索点云
                        kdtreeplus_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/output_kdtreeplus", 10);
                }
       
        private:
                ros::NodeHandle nh;
                ros::Subscriber lidar_sub_;        //订阅原始点云	订阅方
                ros::Subscriber camera_sub_;       //订阅相机发布的二维码点坐标	订阅方
                ros::Publisher kdtree_pc_pub_;     //发布小邻域搜索点云 发布方
                ros::Publisher kdtreeplus_pc_pub_; //发布大邻域搜索点云 发布方
                pcl::PointCloud<RsPointXYZIRT> searchPoint_sub;
                pcl::PointCloud<RsPointXYZIRT> searchPointplus_sub;
                // RsPointXYZIRT  searchPoint;						//邻域搜索圆心

                // 订阅相机发布的二维码中心
                void cloud_cb2(const geometry_msgs::PoseStampedConstPtr &cloud_msg2)
                {
                        center_cam_x = (cloud_msg2->pose.position.z) + 0.10;  // 相机的z方向是雷达的x方向
                        center_cam_y = -(cloud_msg2->pose.position.x) + 0.071; //相机的-x方向是雷达的+y方向
                        center_cam_z =  - 0.12;  //相机的-y方向是雷达的+z方向
                        cout << "相机识别二维码中心："  << "(x = " << center_cam_x << ", y = " << center_cam_y << ", z = " << center_cam_z << " )" << endl;
                        Eigen::Vector3d temp_eulur;
                        temp_eulur = ToEulur(Eigen::Quaterniond(cloud_msg2->pose.orientation.w, cloud_msg2->pose.orientation.x, cloud_msg2->pose.orientation.y, cloud_msg2->pose.orientation.z));
                        cout << "pitch = " << 57.296 * temp_eulur[1] << endl;
                }

                void cloud_cb1(const sensor_msgs::PointCloud2ConstPtr &cloud_msg1)
                {
                        pcl::PointCloud<RsPointXYZIRT>::Ptr original_cloud(new pcl::PointCloud<RsPointXYZIRT>);   //存放雷达原始点云
                        pcl::fromROSMsg(*cloud_msg1, *original_cloud);          //把雷达原始点云从ROS格式转换成RsPointXYZIRT格式并存放到original_cloud里
                        
                        // pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtree(new pcl::PointCloud<RsPointXYZIRT>);     //存储小邻域搜索后的点云
                        // pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtreeplus(new pcl::PointCloud<RsPointXYZIRT>); //存储大邻域搜索后的点云
                        sensor_msgs::PointCloud2 cloud_pt;      //存放要发布的小邻域搜索点云
                        sensor_msgs::PointCloud2 cloud_ptplus;          //存放要发布的大邻域搜索点云
                        // //创建kdtree 结构
                        // pcl::KdTreeFLANN<RsPointXYZIRT> kdtree;
                        // pcl::KdTreeFLANN<RsPointXYZIRT> kdtreeplus;
                        // //传入原始点云用于邻域搜索
                        // kdtree.setInputCloud(original_cloud);
                        // kdtreeplus.setInputCloud(original_cloud);
                        // // 创建两个向量，分别存放近邻的索引值、近邻的中心距
                        // std::vector<int> pointIdxRadiusSearch;              //索引值
                        // std::vector<float> pointRadiusSquaredDistance;      //中心距
                        // std::vector<int> pointIdxRadiusSearch_plus;         //索引值
                        // std::vector<float> pointRadiusSquaredDistance_plus; //中心距
                        // cloud_kdtree->header.frame_id = base_link;     //设置发布点云的坐标系：要设置在输出点云存储的容器中
                        // cloud_kdtreeplus->header.frame_id = base_link; //设置发布点云的坐标系：要设置在输出点云存储的容器中
                        
                        // if (searchPoint.x < 3)
                        // {
                                searchPoint.x = center_cam_x;
                                searchPoint.y = center_cam_y;
                                searchPoint.z = center_cam_z;
                        // }
                        // else
                        // {
                        //         searchPoint.x = center_lidar_x;
                        //         searchPoint.y = center_lidar_y;
                        //         searchPoint.z = center_lidar_z;
                        // }
                        // cout << "搜索中心坐标:(" << searchPoint.x << ", " << searchPoint.y << ", " << searchPoint.z << ") " << endl;
                        // 邻域搜索--->返回搜索到的点数
                        // int num1 = kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                        // int num2 = kdtreeplus.radiusSearch(searchPoint, radiusplus, pointIdxRadiusSearch_plus, pointRadiusSquaredDistance_plus);
                        // for (size_t i = 0; i < num1; ++i)       // 把小邻域搜索到的点放进容器中
                        // {
                        //         cloud_kdtree->push_back(original_cloud->points[pointIdxRadiusSearch[i]]);
                        // }
                        // for (size_t i = 0; i < num2; ++i)       // 把大邻域搜索到的点放进容器中
                        // {
                        //         cloud_kdtreeplus->push_back(original_cloud->points[pointIdxRadiusSearch_plus[i]]);
                        // }
                        // pcl::toROSMsg(*cloud_kdtree, cloud_pt);         //把小邻域搜索到的点云转换成ROS的发布格式
                        // pcl::toROSMsg(*cloud_kdtreeplus, cloud_ptplus); //把大邻域搜索到的点云转换成ROS的发布格式
                        pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtree(new pcl::PointCloud<RsPointXYZIRT>);     //存储小邻域搜索后的点云
                        pcl::CropBox<RsPointXYZIRT> box_filter;//滤波器对象
                        box_filter.setMin(Eigen::Vector4f(searchPoint.x - samll_box_x, searchPoint.y - samll_box_y,  err_z - samll_box_z, 1.0));//Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，齐次坐标）
                        box_filter.setMax(Eigen::Vector4f(searchPoint.x + samll_box_x, searchPoint.y + samll_box_y, err_z + samll_box_z, 1.0));
                        box_filter.setNegative(false);//false是将盒子内的点去除，默认为false
                        box_filter.setInputCloud(original_cloud);//输入源
                        box_filter.filter(*cloud_kdtree);//保留！
                        pcl::toROSMsg(*cloud_kdtree, cloud_pt);         //把小邻域搜索到的点云转换成ROS的发布格式
                        cloud_pt.header.stamp = ros::Time::now();
                        cloud_pt.header.frame_id = base_link;     //设置发布点云的坐标系：要设置在输出点云存储的容器中

                        pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtreeplus(new pcl::PointCloud<RsPointXYZIRT>); //存储大邻域搜索后的点云
                        pcl::CropBox<RsPointXYZIRT> boxplus_filter;//滤波器对象
                        boxplus_filter.setMin(Eigen::Vector4f(searchPoint.x - big_box_x, searchPoint.y - big_box_y,  err_z - big_box_z, 1.0));//Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，齐次坐标）
                        boxplus_filter.setMax(Eigen::Vector4f(searchPoint.x + big_box_x, searchPoint.y + big_box_y, err_z + big_box_z, 1.0));
                        boxplus_filter.setNegative(false);//false是将盒子内的点去除，默认为false
                        boxplus_filter.setInputCloud(original_cloud);//输入源
                        boxplus_filter.filter(*cloud_kdtreeplus);//保留！
                        pcl::toROSMsg(*cloud_kdtreeplus, cloud_ptplus); //把大邻域搜索到的点云转换成ROS的发布格式
                        cloud_ptplus.header.stamp = ros::Time::now();
                        cloud_ptplus.header.frame_id = base_link; //设置发布点云的坐标系：要设置在输出点云存储的容器中
                        
                        kdtree_pc_pub_.publish(cloud_pt);                       //发布小邻域搜索到的点云
                        kdtreeplus_pc_pub_.publish(cloud_ptplus);       //发布大邻域搜索到的点云
                        return;
                }
}; //class kdtree

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
