#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp> 
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#define NUM 8
int main(int argc, char** argv)
{
	vector<cv::Mat> colorImgs, depthImgs; 
	vector<Eigen::Isometry3d> poses; 
	ifstream fin("C:/Users/Mcfun/source/repos/3DR/3DR/pose.txt");
	if (!fin)
	{
		cerr << "cannot find pose file" << endl;
		return 1;
	}
	for (int i = 0; i < NUM; i++)
	{
		boost::format fmt("C:/Users/Mcfun/source/repos/3DR/3DR/%d.%s"); //图像文件格式
		colorImgs.push_back(cv::imread((fmt% (i + 1) % "png").str()));
		depthImgs.push_back(cv::imread((fmt% (i + 1) % "pgm").str(), -1)); // 使用-1读取原始图像

		double data[7] = { 0 };
		for (int i = 0; i < 7; i++)
		{
			fin >> data[i];
		}
		Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
		Eigen::Isometry3d T(q);
		T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
		poses.push_back(T);
	}
	double cx = 633.947449;
	double cy = 404.559906;
	double fx = 682.421509;
	double fy = 682.421509;
	double depthScale = 1000.0;
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;

	PointCloud::Ptr pointCloud(new PointCloud);
	for (int i = 0; i <NUM; i++)
	{
		PointCloud::Ptr current(new PointCloud);
		cout << "处理中: " << i + 1 << endl;
		cv::Mat color = colorImgs[i];
		cv::Mat depth = depthImgs[i];
		Eigen::Isometry3d T = poses[i];
		for (int v = 0; v < color.rows; v++)
			for (int u = 0; u < color.cols; u++)
			{
				unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
				if (d == 0) continue; // 为0表示没有测量到
				if (d >= 38000) continue; // 深度太大时不稳定，去掉
				Eigen::Vector3d point;
				point[2] = double(d) / depthScale;
				point[0] = (u - cx)*point[2] / fx;
				point[1] = (v - cy)*point[2] / fy;
				Eigen::Vector3d pointWorld = T * point;

				PointT p;
				p.x = pointWorld[0];
				p.y = pointWorld[1];
				p.z = pointWorld[2];
				p.b = color.data[v*color.step + u * color.channels()];
				p.g = color.data[v*color.step + u * color.channels() + 1];
				p.r = color.data[v*color.step + u * color.channels() + 2];
				current->points.push_back(p);
			}
		PointCloud::Ptr tmp(new PointCloud);
		pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
		statistical_filter.setMeanK(50);
		statistical_filter.setStddevMulThresh(1.0);
		statistical_filter.setInputCloud(current);
		statistical_filter.filter(*tmp);
		(*pointCloud) += *tmp;
	}
	pointCloud->is_dense = false;
	//pcl::io::savePCDFile("Test1111.pcd", *pointCloud);
	pcl::visualization::PCLVisualizer viewer("室内全景点云三维重建");
	viewer.setCameraPosition(0, 0, -3.0, 0, -1, 0);
	viewer.addCoordinateSystem(0.3);

	viewer.addPointCloud(pointCloud);
	while (!viewer.wasStopped())
		viewer.spinOnce(100);
	// 清除数据并退出
	pointCloud->points.clear();
	system("pause");
	return (0);
}