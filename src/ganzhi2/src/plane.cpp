#include <pcl/search/impl/search.hpp>
 
#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)

#endif // PCL_NO_PRECOMPILE

#include<iostream>
#include <pcl/ModelCoefficients.h>
#include<pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

/*
功能：根据邻域点云拟合平面，绕Z轴旋转原始点云
*/

typedef pcl::PointXYZ PointType;    //旋转平面

void rotate(double A, double B, double C,double D)
{
	cout << "旋转点云" << endl;

	pcl::PointCloud<PointType>::Ptr source_cloud(new pcl::PointCloud<PointType>);
    // 传入原始点云
	pcl::io::loadPCDFile("/home/zh/catkin_ws/src/ganzhi/case1.pcd", *source_cloud) < 0;


	//这里的旋转用的官网里的第二种方法
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	float theta = atan(A/B); // 以弧度表示角度

	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ())); //theta为正时 逆时针旋转，theta为负时 顺时针旋转
	//这里是绕Z轴旋转，绕X轴旋转时改为UnitX，Y轴同理。																		

	/*
    // Print the transformation 打印旋转矩阵
	printf("\nMethod #2: using an Affine3f\n");
	std::cout << transform_2.matrix() << std::endl;
    */

	// Executing the transformation
	pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
	
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

	// Visualization
	printf("Point cloud colors :  white  = original point cloud\n"
		"                        red  = transformed point cloud\n");
	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

	// Define R,G,B colors for the point cloud
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
	// We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}
}


// 拟合平面
void nihe_plane()
{
    double A, B, C, D;//用于接收平面方程的参数
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::PCDReader reader;   //读取PCD
	reader.read("/home/zh/catkin_ws/src/ganzhi/case1.pcd", *cloud);
	//创建一个模型参数对象，用于记录拟合结果
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//inliers通过点云序号来记录模型内点
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	//创建一个分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//Optional,设置结果平面展示的点是分割掉的点还是分割剩下的点
	seg.setOptimizeCoefficients(true);//flase 展示的是分割剩下的点
	//Mandatory-设置目标几何形状
	seg.setModelType(pcl::SACMODEL_PLANE);
	//分割方法：随机采样法
	seg.setMethodType(pcl::SAC_RANSAC);
	//设置误差容忍范围，也就是阈值
	seg.setDistanceThreshold(0.01);
	//输入点云
	seg.setInputCloud(cloud);
	//分割点云
	seg.segment(*inliers, *coefficients);
	
	for (size_t i = 0; i < coefficients->values.size(); ++i)
	{
		cout << "  values[" << i << "]: ";
		cout << "  " << coefficients->values[i] << std::endl;
	}
	A = coefficients->values[0];
	B = coefficients->values[1];
	C = coefficients->values[2];
	D = coefficients->values[3];
	cout << "/*平面方程：Ax+By+Cz+D=0  上面的输出中A=values[0],B=values[1],C=values[2],D=values[3]*/" << std::endl;
    cout << "tan(theta) = " << A/B << std::endl;    //输出tan(theta)
    cout << "theta = " << 180*atan(A/B)/3.14159 << std::endl;   //theta
	rotate(A, B, C, D);     //旋转平面
}

int main()
{
	nihe_plane();
    return 0;
}
