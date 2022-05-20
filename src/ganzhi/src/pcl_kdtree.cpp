#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
using namespace std;

/* 
提取目标点邻域内的点云
输入参数有
    输入点云    string input_filename;
    输出点云    string output_filename;
*/

typedef pcl::PointXYZ PointType;
int main(int argc, char** argv)
{
    string input_filename = "./src/ganzhi/case1.pcd";       //当前路径在/catkin_ws
    string output_filename = "./src/ganzhi/case1_output.pcd";

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr source_key_Neigh(new pcl::PointCloud<PointType>());

    // 读取点云
    pcl::io::loadPCDFile(input_filename, *cloud);
    //创建kdtree 结构
    pcl::KdTreeFLANN<PointType> kdtree;
    //传入点云
    kdtree.setInputCloud(cloud);

    //设置二维码中心点 (关键点)
    PointType searchPoint;
    searchPoint.x = 1.95;
    searchPoint.y = 0.13;
    searchPoint.z = -0.075;

    // 创建两个容器，分别存放近邻的索引值、近邻的中心距
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // 指定搜索半径
    float radius = 0.1;

    std::cout << "Neighbors within radius search at (" << searchPoint.x
            << " " << searchPoint.y
            << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;

    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        cout<<"最近邻点数: "<<pointIdxRadiusSearch.size ()<<endl;
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        {
        //    std::cout << cloud->points[ pointIdxRadiusSearch[i] ].x
        //              << " " << cloud->points[ pointIdxRadiusSearch[i] ].y
        //              << " " << cloud->points[ pointIdxRadiusSearch[i] ].z
        //              << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
            source_key_Neigh->push_back(cloud->points[pointIdxRadiusSearch[i]]);
        }
        pcl::io::savePCDFile(output_filename, *source_key_Neigh);
    }
    else
        cout<<"找不到最近点"<<endl;

    cout<<"end"<<endl;
    return 0;
}
