/*******************************2 激光雷达的数据模型化*******************************/
#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <algorithm>
#include "ganzhi3/plane.h"
#include "ganzhi3/center.h"
#include "ganzhi3/guagou_in_houzhou.h"
#include "ganzhi3/RsPointXYZIRT.h" //定义了速腾的点云结构
#include "/home/zh/catkin_ws/src/ganzhi3/include/ganzhi3/globle_center.h"
using namespace std;

#define RING 80

extern float center_cam_x;
extern float center_cam_y;
extern float center_cam_z;

extern float center_lidar_x;
extern float center_lidar_y;
extern float center_lidar_z;

extern RsPointXYZIRT searchPoint;


// #define rear_axle_center_in_lidar_x            0.9815
// #define rear_axle_center_in_lidar_y            2.5345
// #define hook_in_erweima_x                           0.805
// #define hook_in_erweima_y                           0.765
#define rear_axle_center_in_lidar_x 2.5345
#define rear_axle_center_in_lidar_y -0.9815
#define hook_in_erweima_x 0.765
#define hook_in_erweima_y -0.805
#define pi 3.1415926
ganzhi3::center center;                       //存储要发布的二维中心坐标
ganzhi3::guagou_in_houzhou guagou_in_houzhou; //存储要发布的挂钩相对于后轴中心的坐标

float camera_x; //存放相机识别的二维码中心坐标
float camera_y; //存放相机识别的二维码中心坐标
float lidar_x;  //存放雷达计算的二维码中心坐标
float lidar_y;  //存放雷达计算的二维码中心坐标
float theta;    //存放二维码法向量与挂车轴线的夹角
//定义全局变量用来存储平面拟合的参数
float m1, m2, m3, m4, jiajiao;
static int N_SCANS = 80; //设置激光雷达线束

// 类，原本这个类用来把点云放进各自通道，现在还加入了提取每个通道在棱上的点
class scanID
{
public:
        ros::NodeHandle nh;
        ros::Subscriber kdtreeplus_pc_subscriber_;     //订阅大邻域搜索点云	订阅方
        ros::Subscriber plane_para_subscriber_;        //平面方程参数 订阅方
        ros::Subscriber camera_pub_center_subscriber_; //订阅相机发布的二维码中心
        ros::Publisher center_pub;
        ros::Publisher transform_pub;

        // 构造函数
        scanID()
        {
                // 订阅平面方程的参数，并赋值给本节点中的全局变量
                plane_para_subscriber_ = nh.subscribe<ganzhi3::plane>("/plane_para", 1, &scanID::callback2, this);
                // ！！！如果回调函数不是类内函数用下面这种形式！！！
                // plane_para_sub_ = nh.subscribe<ganzhi3::plane> ("/plane_para", 1, callback2);      //回调函数是不是类的成员
                // 订阅到大邻域搜索到的点云后，进入回调函数进行计算
                kdtreeplus_pc_subscriber_ = nh.subscribe<sensor_msgs::PointCloud2>("/output_kdtreeplus", 10, &scanID::callback1, this);

                center_pub = nh.advertise<ganzhi3::center>("/search_center", 10);
                transform_pub = nh.advertise<ganzhi3::guagou_in_houzhou>("/transform", 10);
        }
        float point_plane_distance(const RsPointXYZIRT &point);                               //点到平面的距离计算函数
        float point_point_distance(const RsPointXYZIRT &point1, const RsPointXYZIRT &point2); //点到点的距离计算函数

        void callback1(const sensor_msgs::PointCloud2ConstPtr &cloud_msg); // 订阅大邻域搜索点云的回调函数
        void callback2(const ganzhi3::plane::ConstPtr &plane);             //订阅平面参数的回调函数

private:
        float fenzi;                    //距离公式的分子
        float fenmu;               //距离公式的分母
        float point_to_plane_dist; //点到平面距离
        float point_to_point_dist; //点到点距离
        //存放平面上点云的容器
        vector<RsPointXYZIRT> POINT_in_plane;

        // 设定排序方式，按旋转角从小到大排序（我感觉应该从汽车侧向的-180°到纵向的-90°再到侧向的0°排序比较合理）
        class timegreater
        {
        public:
                bool operator()(RsPointXYZIRT point1, RsPointXYZIRT point2)
                {
                        return point1.y < point2.y;
                }
        };
};

// 订阅大邻域搜索点云的回调函数_类外实现
void scanID::callback1(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
        pcl::PointCloud<RsPointXYZIRT>::Ptr kdtreeplus_pointcloud(new pcl::PointCloud<RsPointXYZIRT>); //存储大邻域传进来的点云
        pcl::fromROSMsg(*cloud_msg, *kdtreeplus_pointcloud);

        std::vector<pcl::PointCloud<RsPointXYZIRT>> laserRing(RING); //定义雷达线束，动态数组集合
        int cloudsize = (*kdtreeplus_pointcloud).points.size();
        for (int i = 0; i < cloudsize; i++)
        {
                laserRing[kdtreeplus_pointcloud->points[i].ring].push_back(kdtreeplus_pointcloud->points[i]); // 将每个点放入对应的帧通道中
        }

        float average_X = 0, average_Y = 0; //二维码平均中心坐标
        int calculate_N = 0;                               //计数器，记录二维码平面上的线束
        for (int i = 0; i < N_SCANS; i++)                  // N_SCANS个通道逐一遍历
        {
                // int calculate_N_every_scan = 0;
                if (!laserRing[i].empty()) //判断该通道中是否有点云
                {
                        // 给该通道中的所有点按水平旋转角从小到大排列
                        sort(laserRing[i].begin(), laserRing[i].end(), timegreater());

                        float plane_distance; //点到平面的距离
                        float point_distance; //用两个边点的距离表示棱的距离
                        int laserRing_num = laserRing[i].size();
                        for (int j = 0; j < laserRing_num; ++j) //计算点到平面的距离
                        {
                                //  调用点到平面距离计算公式，计算每个点到平面的距离是否在阈值内来判断是否是平面点
                                plane_distance = point_plane_distance(laserRing[i].points[j]);
                                //判断点是否在平面上
                                if (plane_distance > -0.05 && plane_distance < 0.05) //点到平面的距离阈值设为5cm
                                {
                                        POINT_in_plane.push_back(laserRing[i].points[j]);
                                }
                        }

                        if (!POINT_in_plane.empty())
                        {
                                int number_POINT_in_plane = POINT_in_plane.size(); //平面上的点数
                                point_distance = point_point_distance(POINT_in_plane[1], POINT_in_plane[number_POINT_in_plane - 1]);
                                if (point_distance > 0.55 && point_distance < 0.65)     //板长60cm
                                {
                                        average_X += (POINT_in_plane[1].x + POINT_in_plane[number_POINT_in_plane - 1].x) / 2;
                                        average_Y += (POINT_in_plane[1].y + POINT_in_plane[number_POINT_in_plane - 1].y) / 2;
                                        calculate_N++;
                                        // calculate_N_every_scan++;
                                }
                        }
                        POINT_in_plane.clear(); // 清空数组
                }
        }
        lidar_x = average_X / calculate_N;
        lidar_y = average_Y / calculate_N;

        cout << "二维码平面上总共有：" << calculate_N << "条激光雷达点云线束" << endl;
        cout << "二维码中心雷达计算坐标为：(" << lidar_x << "," << lidar_y << ")" << endl;
}

// 订阅平面方程的参数的回调函数_类外实现
void scanID::callback2(const ganzhi3::plane::ConstPtr &plane)
{
        m1 = plane->a;
        m2 = plane->b;
        m3 = plane->c;
        m4 = plane->d;
        jiajiao = plane->jiajiao;
}

// void scanID::callback3(const geometry_msgs::PoseStampedConstPtr& cloud_msg2){
//     camera_x  = (cloud_msg2->pose.position.z) + 0.040;
//     camera_y = -(cloud_msg2->pose.position.x) + 0.071;
// }

// 点到平面距离函数的类外实现
float scanID::point_plane_distance(const RsPointXYZIRT &point)
{
        fenzi = m1 * point.x + m2 * point.y + m3 * point.z + m4;
        fenmu = sqrt(m1 * m1 + m2 * m2 + m3 * m3);
        point_to_plane_dist = fenzi / fenmu;
        return point_to_plane_dist;
}

// 点到点距离函数的类外实现
float scanID::point_point_distance(const RsPointXYZIRT &point1, const RsPointXYZIRT &point2)
{
        point_to_point_dist = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y) + (point1.z - point2.z) * (point1.z - point2.z));
        return point_to_point_dist;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "scanid");
        scanID scanid;                          // 实例化一个scanID类
        
        ros::Rate loop_rate(10);        //循环发布频率为10HZ
        while (ros::ok())
        {
                if (center_cam_x < 1.7)
                {
                        center.c_x = center_cam_x;
                        center.c_y = center_cam_y;
                        center.c_theta = 0;
                }
                else if (lidar_x != lidar_x) //雷达计算为nan时继续上次的数据
                {
                        center.c_x = center.c_x;
                        center.c_y = center.c_y;
                        center.c_theta = center.c_theta;
                }
                else
                {
                        center.c_x = lidar_x;
                        center.c_y = lidar_y;
                        center.c_theta = jiajiao;
                }
                center_lidar_x = center.c_x;
                center_lidar_y = center.c_y;
                center_lidar_z = center_cam_z;
                //都在雷达坐标系下坐标转换（WWJ版）
                // guagou_in_houzhou.guagou_in_houzhouzhongxin_x = hook_in_erweima_y * sin(theta) + hook_in_erweima_x * cos(theta) + center.c_x - rear_axle_center_in_lidar_x;
                // guagou_in_houzhou.guagou_in_houzhouzhongxin_y = hook_in_erweima_y * cos(theta) - hook_in_erweima_x * sin(theta) + center.c_y - rear_axle_center_in_lidar_y;
                guagou_in_houzhou.guagou_in_houzhouzhongxin_x = hook_in_erweima_y * sin(center.c_theta) + hook_in_erweima_x * cos(center.c_theta) + searchPoint.x - rear_axle_center_in_lidar_x;
                guagou_in_houzhou.guagou_in_houzhouzhongxin_y = hook_in_erweima_y * cos(center.c_theta) - hook_in_erweima_x * sin(center.c_theta) + searchPoint.y - rear_axle_center_in_lidar_y;
                scanid.center_pub.publish(center);      // 发布计算得到的二维中心坐标
                scanid.transform_pub.publish(guagou_in_houzhou);        // 发布挂钩在后轴中心下的坐标
                ros::spinOnce();
                loop_rate.sleep();
        }
        return 0;
}