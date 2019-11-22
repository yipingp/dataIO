/*
*Copyright: Copyright (c) 2019
*Author: Yiping Peng
*Date: 2019-08-22
*Site: https://github.com/timeperiod/dataIO
*Description: C++ template class of CV data IO
*/

#pragma once
#ifndef _DATAIO_WITHPCL
#define _DATAIO_WITHPCL

#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "dataIO.hpp"

using namespace std;

template <class PointType = pcl::PointXYZ>
class PointCloudIO: public dataIO
{
public:
	// ToDo: 把点云的放到一个子类，这样用不到点云的情况就不用include PCL的头文件了，可以减少编译时间
	// point cloud io

	pcl::PointCloud<PointType> readPC();
	void savePC(pcl::PointCloud<PointType> cloud, string addName = "", string suffix = "");

	// point cloud rendering

	// background color  default: black
	pcl::PointXYZ rgb_bkg = pcl::PointXYZ(0, 0, 0);
	// text color        defalut: white
	pcl::PointXYZ rgb_txt_v1 = pcl::PointXYZ(1, 1, 1); pcl::PointXYZ rgb_txt_v2 = pcl::PointXYZ(1, 1, 1); pcl::PointXYZ rgb_txt_v3 = pcl::PointXYZ(1, 1, 1); pcl::PointXYZ rgb_txt_v4 = pcl::PointXYZ(1, 1, 1);
	void set_txt_rgb(pcl::PointXYZ _rgb_txt_v1, pcl::PointXYZ _rgb_txt_v2 = pcl::PointXYZ(1, 1, 1), pcl::PointXYZ _rgb_txt_v3 = pcl::PointXYZ(1, 1, 1), pcl::PointXYZ _rgb_txt_v4 = pcl::PointXYZ(1, 1, 1));
	// text content
	string txt_v1 = "cloud1"; string txt_v2 = "cloud2"; string txt_v3 = "cloud3"; string txt_v4 = "cloud4";
	void set_txt_content(string _txt_v1, string _txt_v2 = "cloud2", string _txt_v3 = "cloud3", string _txt_v4 = "cloud4");
	// scale of coordinate  default: 0.1m
	double coordinateScale = 0.1;

	void PCrender(pcl::PointCloud<PointType> cloudView1, pcl::PointCloud<PointType> cloudView2 = pcl::PointCloud<PointType>(), pcl::PointCloud<PointType> cloudView3 = pcl::PointCloud<PointType>(), pcl::PointCloud<PointType> cloudView4 = pcl::PointCloud<PointType>());
};

// point cloud
template <class PointType>
pcl::PointCloud<PointType> PointCloudIO<PointType>::readPC()
{
	string openFilename = path + filename;
	string cut = openFilename.substr(openFilename.find_last_of(".") + 1);
	pcl::PointCloud<PointType> cloud;

	if (cut == "ply")
		pcl::io::loadPLYFile(openFilename, cloud); // Load .ply File
	if (cut == "pcd")
		pcl::io::loadPCDFile(openFilename, cloud); // Load .pcd File

	return cloud;
}

/*
@param cloud Cloud you want to save.
@param addName Name that will be insert into the end of file's pure name and suffix name.
@param suffix The suffix can be set, in case member variable "filename" is not a pcl point cloud file name,
*/
template <class PointType>
void PointCloudIO<PointType>::savePC(pcl::PointCloud<PointType> cloud, string addName, string suffix)
{
	string pureName = filename.substr(0, filename.rfind("."));
	if (suffix == "")
		suffix = filename.substr(filename.find_last_of('.'));
	if (savePath == "")
		savePath = path;
	string openFilename = savePath + pureName + addName + suffix;

	if (suffix == ".ply")
		pcl::io::savePLYFileASCII(openFilename, cloud);
	if (suffix == ".pcd")
		pcl::io::savePCDFileASCII(openFilename, cloud);
}


// point cloud rendering

/*
@param cloudView1 Point cloud you want view in viewport v1.
@param cloudView2 Point cloud you want view in viewport v2.
@param cloudView3 Point cloud you want view in viewport v3.
@param cloudView4 Point cloud you want view in viewport v4.

@note The number of parameters must be four, so fill empty point cloud if necessary.
*/
template <class PointType>
void PointCloudIO<PointType>::PCrender(pcl::PointCloud<PointType> cloudView1, pcl::PointCloud<PointType> cloudView2, pcl::PointCloud<PointType> cloudView3, pcl::PointCloud<PointType> cloudView4)
{
	// 创建viewport
	int v1 = 0;
	int v2 = 1;
	int v3 = 2;
	int v4 = 3;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PC Visualizer"));
	// 一个点云的情况
	if (cloudView2.size() == 0)
	{
		// 转换成pcl::PointCloud::Ptr形式
		typename pcl::PointCloud<PointType>::Ptr cloud_Ptr1(new pcl::PointCloud<PointType>);
		cloud_Ptr1 = cloudView1.makeShared();
		viewer->setBackgroundColor(rgb_bkg.x, rgb_bkg.y, rgb_bkg.z);
		viewer->addPointCloud<PointType>(cloud_Ptr1, "Cloud");
		// addText("string",x坐标，y坐标，字号，"id",r，g，b)  // 1/1/1是白色字体
		viewer->addText(txt_v1, 10, 15, 16, rgb_txt_v1.x, rgb_txt_v1.y, rgb_txt_v1.z, "cloud");
		viewer->addCoordinateSystem(coordinateScale);
		viewer->initCameraParameters();  // Camera Parameters for ease of viewing

		viewer->spin(); // Allow user to rotate point cloud and view it
	}
	// 两个点云的情况
	if (cloudView2.size() != 0 && cloudView3.size() == 0)
	{
		typename pcl::PointCloud<PointType>::Ptr cloud_Ptr1(new pcl::PointCloud<PointType>); cloud_Ptr1 = cloudView1.makeShared();
		typename pcl::PointCloud<PointType>::Ptr cloud_Ptr2(new pcl::PointCloud<PointType>); cloud_Ptr2 = cloudView2.makeShared();

		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		// 背景颜色设置要在viewport创建完成之后才有效
		viewer->setBackgroundColor(rgb_bkg.x, rgb_bkg.y, rgb_bkg.z);
		viewer->addPointCloud<PointType>(cloud_Ptr1, "Cloud1", v1);
		viewer->addPointCloud<PointType>(cloud_Ptr2, "Cloud2", v2);

		viewer->addText(txt_v1, 10, 15, 16, rgb_txt_v1.x, rgb_txt_v1.y, rgb_txt_v1.z, "cloud1", v1);
		viewer->addText(txt_v2, 10, 15, 16, rgb_txt_v2.x, rgb_txt_v2.y, rgb_txt_v2.z, "cloud2", v2);
		viewer->addCoordinateSystem(coordinateScale);
		viewer->initCameraParameters();

		viewer->spin();
	}
	// 三个点云的情况
	if (cloudView3.size() != 0 && cloudView4.size() == 0)
	{
		typename pcl::PointCloud<PointType>::Ptr cloud_Ptr1(new pcl::PointCloud<PointType>); cloud_Ptr1 = cloudView1.makeShared();
		typename pcl::PointCloud<PointType>::Ptr cloud_Ptr2(new pcl::PointCloud<PointType>); cloud_Ptr2 = cloudView2.makeShared();
		typename pcl::PointCloud<PointType>::Ptr cloud_Ptr3(new pcl::PointCloud<PointType>); cloud_Ptr3 = cloudView3.makeShared();

		viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
		viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
		viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v3);

		viewer->setBackgroundColor(rgb_bkg.x, rgb_bkg.y, rgb_bkg.z);
		viewer->addPointCloud<PointType>(cloud_Ptr1, "Cloud1", v1);
		viewer->addPointCloud<PointType>(cloud_Ptr2, "Cloud2", v2);
		viewer->addPointCloud<PointType>(cloud_Ptr3, "Cloud3", v3);

		viewer->addText(txt_v1, 10, 15, 16, rgb_txt_v1.x, rgb_txt_v1.y, rgb_txt_v1.z, "cloud1", v1);
		viewer->addText(txt_v2, 10, 15, 16, rgb_txt_v2.x, rgb_txt_v2.y, rgb_txt_v2.z, "cloud2", v2);
		viewer->addText(txt_v3, 10, 15, 16, rgb_txt_v3.x, rgb_txt_v3.y, rgb_txt_v3.z, "cloud3", v3);
		viewer->addCoordinateSystem(coordinateScale);
		viewer->initCameraParameters();

		viewer->spin(); // Allow user to rotate point cloud and view it
	}
	// 四个点云的情况
	if (cloudView4.size() != 0)
	{
		typename pcl::PointCloud<PointType>::Ptr cloud_Ptr1(new pcl::PointCloud<PointType>); cloud_Ptr1 = cloudView1.makeShared();
		typename pcl::PointCloud<PointType>::Ptr cloud_Ptr2(new pcl::PointCloud<PointType>); cloud_Ptr2 = cloudView2.makeShared();
		typename pcl::PointCloud<PointType>::Ptr cloud_Ptr3(new pcl::PointCloud<PointType>); cloud_Ptr3 = cloudView3.makeShared();
		typename pcl::PointCloud<PointType>::Ptr cloud_Ptr4(new pcl::PointCloud<PointType>); cloud_Ptr4 = cloudView4.makeShared();

		viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
		viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
		viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v3);
		viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v4);

		viewer->setBackgroundColor(rgb_bkg.x, rgb_bkg.y, rgb_bkg.z);
		viewer->addPointCloud<PointType>(cloud_Ptr1, "Cloud1", v1);
		viewer->addPointCloud<PointType>(cloud_Ptr2, "Cloud2", v2);
		viewer->addPointCloud<PointType>(cloud_Ptr3, "Cloud3", v3);
		viewer->addPointCloud<PointType>(cloud_Ptr4, "Cloud4", v4);

		viewer->addText(txt_v1, 10, 15, 16, rgb_txt_v1.x, rgb_txt_v1.y, rgb_txt_v1.z, "cloud1", v1);
		viewer->addText(txt_v2, 10, 15, 16, rgb_txt_v2.x, rgb_txt_v2.y, rgb_txt_v2.z, "cloud2", v2);
		viewer->addText(txt_v3, 10, 15, 16, rgb_txt_v3.x, rgb_txt_v3.y, rgb_txt_v3.z, "cloud3", v3);
		viewer->addText(txt_v4, 10, 15, 16, rgb_txt_v4.x, rgb_txt_v4.y, rgb_txt_v4.z, "cloud4", v4);
		viewer->addCoordinateSystem(coordinateScale);
		viewer->initCameraParameters();

		viewer->spin();
	}
}

/*
@param _rgb_txt_v1 The color of text in viewport v1
@param _rgb_txt_v2 The color of text in viewport v2
@param _rgb_txt_v3 The color of text in viewport v3
@param _rgb_txt_v4 The color of text in viewport v4

@note The color of text will remain as defulat(white) if not set.
*/
template <class PointType>
void PointCloudIO<PointType>::set_txt_rgb(pcl::PointXYZ _rgb_txt_v1, pcl::PointXYZ _rgb_txt_v2, pcl::PointXYZ _rgb_txt_v3, pcl::PointXYZ _rgb_txt_v4)
{
	rgb_txt_v1 = _rgb_txt_v1;
	rgb_txt_v2 = _rgb_txt_v2;
	rgb_txt_v3 = _rgb_txt_v3;
	rgb_txt_v4 = _rgb_txt_v4;
}

/*
@param _txt_v1 The color of text in viewport v1
@param _txt_v2 The color of text in viewport v2
@param _txt_v3 The color of text in viewport v3
@param _txt_v4 The color of text in viewport v4

@note The text will remain as defulat(cloud1,cloud2,cloud3,cloud4,respectively) if not set.
*/
template <class PointType>
void PointCloudIO<PointType>::set_txt_content(string _txt_v1, string _txt_v2, string _txt_v3, string _txt_v4)
{
	txt_v1 = _txt_v1;
	txt_v2 = _txt_v2;
	txt_v3 = _txt_v3;
	txt_v4 = _txt_v4;
}


#endif
