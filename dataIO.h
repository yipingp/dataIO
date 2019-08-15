#include <iostream>
#include <string>
#include <vector>

#include <io.h>

#include <opencv2/opencv.hpp>


#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>




#ifndef _DATAIO
#define _DATAIO

using namespace std;

template <class PointType=pcl::PointXYZ>
class dataIO
{
public:
	string path;
	string savePath;

	// txt

	string txtFilename;

	int save2dPts(vector<cv::Point2f> &Data, string addName);
	int save3dPts(vector<cv::Point3f> &Data, string addName);
	vector<cv::Point2f> read2dPts();
	vector<cv::Point3f> read3dPts();

	// files

	// file names with path
	vector<string> files;
	// file names without path
	vector<string> ownnames;
	void getAllFiles();

	// image

	string imgFilename;
	cv::Mat readImg(int flags = CV_LOAD_IMAGE_UNCHANGED);
	bool saveImg(cv::Mat img, string addName);

	// point cloud io

	string pcFilename;

	pcl::PointCloud<PointType> readPC();
	void savePC(pcl::PointCloud<PointType> cloud, string addName);

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


	void PCrender(pcl::PointCloud<PointType> cloudView1, pcl::PointCloud<PointType> cloudView2, pcl::PointCloud<PointType> cloudView3, pcl::PointCloud<PointType> cloudView4);
	

	
};

// txt
/*
@param Data 2d points data you want to save.
@param addName Name that will be insert into the end of file name and suffix name.
*/
template <class PointType>
int dataIO<PointType>::save2dPts(vector<cv::Point2f> &Data, string addName)
{
	string pureName = txtFilename.substr(0, txtFilename.rfind(".")); // 获取不带后缀名的纯文件名 // rfind = reverse find
	string suffix = ".txt";
	if (savePath == "")
		savePath = path;
	string openFilename = savePath + pureName + addName + suffix;
	// 以只读模式创建新文件
	fstream openTXT(openFilename, fstream::out);
	openTXT.close();

	fstream writeTXT;
	writeTXT.open(openFilename);
	assert(writeTXT.is_open());   //若失败,则输出错误消息,并终止程序运行
	for (int i = 0; i < Data.size(); i++)
	{
		if (Data[i].x != 0)
		{
			writeTXT << Data[i].x << " " << Data[i].y << " " << endl;
		}
	}
	writeTXT.close();
	return 0;
}

/*
@param Data 3d points data you want to save.
@param addName Name that will be insert into the end of file name and suffix name.
*/
template <class PointType>
int dataIO<PointType>::save3dPts(vector<cv::Point3f> &Data,string addName)
{
	string pureName = txtFilename.substr(0, txtFilename.rfind("."));
	string suffix = ".txt";
	if (savePath == "")
		savePath = path;
	string openFilename = savePath + pureName + addName + suffix;

	fstream openTXT(openFilename, fstream::out);
	openTXT.close();

	fstream writeTXT;
	writeTXT.open(openFilename);
	assert(writeTXT.is_open());   //若失败,则输出错误消息,并终止程序运行
	for (int i = 0; i < Data.size(); i++)
	{
		if (Data[i].x != 0)
		{
			writeTXT << Data[i].x << " " << Data[i].y << " " << Data[i].z << endl;
		}
	}
	writeTXT.close();
	return 0;
}

template <class PointType>
vector<cv::Point2f> dataIO<PointType>::read2dPts()
{
	string openFilename = path + txtFilename;
	vector<cv::Point2f> pts;
	ifstream readTXT;
	readTXT.open(openFilename.data());   //将文件流对象与文件连接起来 
	assert(readTXT.is_open());   //若失败,则输出错误消息,并终止程序运行 
	cv::Point2f point;

	while (!readTXT.eof())
	{
		readTXT >> point.x >> point.y;
		pts.push_back(point);
	}
	pts.erase(pts.begin() + pts.size() - 1); // 去除重复读取的最后一行
	readTXT.close();             //关闭文件输入流 
	return pts;
}

template <class PointType>
vector<cv::Point3f> dataIO<PointType>::read3dPts()
{
	string openFilename = path + txtFilename;
	vector<cv::Point3f> pts;
	ifstream readTXT;
	readTXT.open(openFilename.data());   //将文件流对象与文件连接起来 
	assert(readTXT.is_open());   //若失败,则输出错误消息,并终止程序运行 
	cv::Point3f point;

	while (!readTXT.eof())
	{
		readTXT >> point.x >> point.y >> point.z;
		pts.push_back(point);
	}
	pts.erase(pts.begin() + pts.size() - 1); // 去除重复读取的最后一行
	readTXT.close();             //关闭文件输入流 
	return pts;
}

//files
/* 
* https://www.cnblogs.com/yuehouse/p/10159358.html
* path为文件夹路径
* files存储文件的路径及名称
* ownname只存储文件的名称
*/
void GetAllFiles(string path, vector<string>& files, vector<string> &ownname)
{
	//文件句柄  
	intptr_t    hFile = 0; // win10
	//文件信息  
	struct _finddata_t fileinfo;
	string p;
	// "\\*"是指读取文件夹下的所有类型的文件，若想读取特定类型的文件，以png为例，则用“\\*.png”
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之  
			//如果不是,加入列表  
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					GetAllFiles(p.assign(path).append("\\").append(fileinfo.name), files, ownname);
			}
			else
			{
				files.push_back(path + "\\" + fileinfo.name);
				ownname.push_back(fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

/*
* Get all files in path
*/
template <class PointType>
void dataIO<PointType>::getAllFiles()
{
	GetAllFiles(path, files, ownnames);
}

// image
/*
@param flags Flag that can take values of cv::ImreadModes.
*/
template <class PointType>
cv::Mat dataIO<PointType>::readImg(int flags)
{
	string openFilename = path + imgFilename;
	cv::Mat img = cv::imread(openFilename,flags);
	return img;
}

/*
@param img Image you want to save.
@param addName Name that will be insert into the end of file name and suffix name.
*/
template <class PointType>
bool dataIO<PointType>::saveImg(cv::Mat img, string addName)
{
	string pureName = imgFilename.substr(0, imgFilename.rfind("."));     // 获取不带后缀名的纯文件名 // rfind = reverse find
	string suffix = imgFilename.substr(imgFilename.find_last_of('.') );  // 获取后缀名（获取txtFilename从最后一个点的位置开始直到最后的字符）
	if (savePath == "")
		savePath = path;
	string openFilename = savePath + pureName + addName + suffix;
	return cv::imwrite(openFilename, img);
}


// point cloud
template <class PointType>
pcl::PointCloud<PointType> dataIO<PointType>::readPC()
{
	string openFilename = path + pcFilename;
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
@param addName Name that will be insert into the end of file name and suffix name.
*/
template <class PointType>
void dataIO<PointType>::savePC(pcl::PointCloud<PointType> cloud, string addName)
{
	string pureName = pcFilename.substr(0, pcFilename.rfind("."));
	string suffix = pcFilename.substr(pcFilename.find_last_of('.') );
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
void dataIO<PointType>::PCrender(pcl::PointCloud<PointType> cloudView1, pcl::PointCloud<PointType> cloudView2, pcl::PointCloud<PointType> cloudView3, pcl::PointCloud<PointType> cloudView4)
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
		pcl::PointCloud<PointType>::Ptr cloud_Ptr1(new pcl::PointCloud<PointType>); cloud_Ptr1 = cloudView1.makeShared(); 
		viewer->setBackgroundColor(rgb_bkg.x, rgb_bkg.y, rgb_bkg.z);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_Ptr1, "Cloud");
		// addText("string",x坐标，y坐标，字号，"id",r，g，b)  // 1/1/1是白色字体
		viewer->addText(txt_v1, 10, 15, 16, rgb_txt_v1.x, rgb_txt_v1.y, rgb_txt_v1.z, "cloud");
		viewer->addCoordinateSystem(coordinateScale);
		viewer->initCameraParameters();  // Camera Parameters for ease of viewing

		viewer->spin(); // Allow user to rotate point cloud and view it
	}
	// 两个点云的情况
	if (cloudView2.size() != 0 && cloudView3.size() == 0)
	{
		pcl::PointCloud<PointType>::Ptr cloud_Ptr1(new pcl::PointCloud<PointType>); cloud_Ptr1 = cloudView1.makeShared();
		pcl::PointCloud<PointType>::Ptr cloud_Ptr2(new pcl::PointCloud<PointType>); cloud_Ptr2 = cloudView2.makeShared();

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
		pcl::PointCloud<PointType>::Ptr cloud_Ptr1(new pcl::PointCloud<PointType>); cloud_Ptr1 = cloudView1.makeShared();
		pcl::PointCloud<PointType>::Ptr cloud_Ptr2(new pcl::PointCloud<PointType>); cloud_Ptr2 = cloudView2.makeShared();
		pcl::PointCloud<PointType>::Ptr cloud_Ptr3(new pcl::PointCloud<PointType>); cloud_Ptr3 = cloudView3.makeShared();

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
		pcl::PointCloud<PointType>::Ptr cloud_Ptr1(new pcl::PointCloud<PointType>); cloud_Ptr1 = cloudView1.makeShared();
		pcl::PointCloud<PointType>::Ptr cloud_Ptr2(new pcl::PointCloud<PointType>); cloud_Ptr2 = cloudView2.makeShared();
		pcl::PointCloud<PointType>::Ptr cloud_Ptr3(new pcl::PointCloud<PointType>); cloud_Ptr3 = cloudView3.makeShared();
		pcl::PointCloud<PointType>::Ptr cloud_Ptr4(new pcl::PointCloud<PointType>); cloud_Ptr4 = cloudView4.makeShared();

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
void dataIO<PointType>::set_txt_rgb(pcl::PointXYZ _rgb_txt_v1, pcl::PointXYZ _rgb_txt_v2, pcl::PointXYZ _rgb_txt_v3, pcl::PointXYZ _rgb_txt_v4)
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
void dataIO<PointType>::set_txt_content(string _txt_v1, string _txt_v2, string _txt_v3, string _txt_v4)
{
	txt_v1 = _txt_v1;
	txt_v2 = _txt_v2;
	txt_v3 = _txt_v3;
	txt_v4 = _txt_v4;
}


#endif
