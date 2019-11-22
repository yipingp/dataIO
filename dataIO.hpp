 /*
 *Copyright: Copyright (c) 2019
 *Author: Yiping Peng
 *Date: 2019-08-22
 *Site: https://github.com/timeperiod/dataIO
 *Description: C++ template class of CV data IO
 */ 

#pragma once
#ifndef _DATAIO
#define _DATAIO

#include <string>
#include <vector>
#include <direct.h>

#include <io.h>

#include <opencv2/opencv.hpp>

using namespace std;

class dataIO
{
public:
	//~dataIO();
	// TODO：在函数里面判断path和savepath最后有没有//或\，没有的话补上\
	// 文件路径
	string path;
	// 存储路径，如果没有设置，则保持和path相同
	string savePath;

	// txt
	// 文件名，不管是读取还是保存都需要设置（除了save_RGBD_dataset()不需要外）
	string filename;

	int savePts(vector<cv::Point2f> &Data, string addName = "");
	int savePts(vector<cv::Point3f> &Data, string addName = "");
	vector<cv::Point2f> read2dPts(string addName = "");
	vector<cv::Point3f> read3dPts(string addName = "");

	int save_RGBD_dataset(string folderOfDataset, vector<string> filenamesOfRGBs, vector<string> filenamesOfDepths);

	// files

	// file names with path
	vector<string> filepaths;
	// file names without path
	vector<string> ownnames;
	void getAllFiles();

	// image
	int frameIdx = 0;
	vector<string> fileList;
	cv::Mat readImg(int flags = cv::IMREAD_UNCHANGED);
	bool saveImg(cv::Mat img, string addName = "", string suffix = "");
	cv::Mat getImage();
	cv::Mat getDepth();

protected:
	// image
	vector<string> readFromDataset(std::string fileList);
	// files
	void GetAllFiles(string path, vector<string>& filepaths, vector<string> &ownname);
};

// txt
/*
@param Data 2d points data you want to save.
@param addName Name that will be insert into the end of file's pure name and suffix name.
*/

int dataIO::savePts(vector<cv::Point2f> &Data, string addName)
{
	string pureName = filename.substr(0, filename.rfind(".")); // 获取不带后缀名的纯文件名 // rfind = reverse find
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
@param addName Name that will be insert into the end of file's pure name and suffix name.
*/
int dataIO::savePts(vector<cv::Point3f> &Data,string addName)
{
	string pureName = filename.substr(0, filename.rfind("."));
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

/*
@Breif 
Create a folder named folderOfDataset following in member variable path or savePath, in which consists of depth.txt, rgb.txt, as well as folders named depth and rgb repectively.
@param folderOfDataset Folder's name of the dataset to be created.
@param filenamesOfRGBs file paths of RGB images
@param filenamesOfDepths file paths of depth images
*/
int dataIO::save_RGBD_dataset(string folderOfDataset, vector<string> filenamesOfRGBs,vector<string> filenamesOfDepths)
{
	if (savePath == "")
		savePath = path;
	string openFilename;
	// 最后一位是savePath.length - 1，因为从0开始
	if (savePath.rfind("/") != savePath.length() - 1 && savePath.rfind("\\") != savePath.length() - 1)// 路径savePath没带\\[和]/的情况
		openFilename = savePath + "/" + folderOfDataset;
	else
		openFilename = savePath + folderOfDataset;

	// 创建文件夹
	string temp = openFilename;
	_mkdir(temp.data()); // 在路径savePath下创建folderOfDataset
	temp = openFilename + "/depth";
	_mkdir(temp.data());
	temp = openFilename + "/rgb";
	_mkdir(temp.data());

	/****写到folderOfDataset文件夹下的depth.txt*****/
	fstream openDepthTXT(openFilename + "/depth.txt" , fstream::out);
	openDepthTXT.close();

	fstream writeDepthTXT;
	writeDepthTXT.open(openFilename + "/depth.txt");
	assert(writeDepthTXT.is_open());   //若失败,则输出错误消息,并终止程序运行
	writeDepthTXT << "# depth maps" << endl;
	writeDepthTXT << "# file: '" + folderOfDataset + ".bag'" << endl;
	writeDepthTXT << "# timestamp filename" << endl;
	int idx1, idx2, idx3;
	for (int i = 0; i < filenamesOfDepths.size(); i++)
	{
		// substr第二个参数是长度不是截止位置
		idx1 = filenamesOfDepths[i].rfind("/") + 1;
		idx2 = filenamesOfDepths[i].rfind(".");
		idx3 = filenamesOfDepths[i].rfind("th/") - 3;
		// e.g. 1_depth depth/1_depth.png
		writeDepthTXT << filenamesOfDepths[i].substr(idx1, idx2-idx1) + " " + filenamesOfDepths[i].substr(idx3) << endl;
	}
	writeDepthTXT.close();

	/****写到folderOfDataset文件夹下的rgb.txt*****/
	fstream openRgbTXT(openFilename + "/rgb.txt" , fstream::out);
	openRgbTXT.close();

	fstream writeRgbTXT;
	writeRgbTXT.open(openFilename + "/rgb.txt");
	assert(writeRgbTXT.is_open());   //若失败,则输出错误消息,并终止程序运行
	writeRgbTXT << "# color images" << endl;
	writeRgbTXT << "# file: '" + folderOfDataset + ".bag'" << endl;
	writeRgbTXT << "# timestamp filename" << endl;
	for (int i = 0; i < filenamesOfDepths.size(); i++)
	{
		idx1 = filenamesOfRGBs[i].rfind("/") + 1;
		idx2 = filenamesOfRGBs[i].rfind(".");
		idx3 = filenamesOfRGBs[i].rfind("gb/") - 1;
		// e.g. 1_color rgb/1_colo.png
		writeRgbTXT << filenamesOfRGBs[i].substr(idx1, idx2 - idx1) + " " + filenamesOfRGBs[i].substr(idx3) << endl;
	}
	writeRgbTXT.close();
	return 0;
}

/*
@param addName Name that will be insert into the end of file's pure name and suffix name.
*/
vector<cv::Point2f> dataIO::read2dPts(string addName)
{
	string pureName = filename.substr(0, filename.rfind("."));
	string suffix = ".txt";
	string openFilename = path + pureName + addName + suffix;
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

/*
@param addName Name that will be insert into the end of file's pure name and suffix name.
*/
vector<cv::Point3f> dataIO::read3dPts(string addName)
{
	string pureName = filename.substr(0, filename.rfind("."));
	string suffix = ".txt";
	string openFilename = path + pureName + addName + suffix;
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
* filepaths存储文件的路径及名称
* ownname只存储文件的名称
*/
void dataIO::GetAllFiles(string path, vector<string>& filepaths, vector<string> &ownname)
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
					GetAllFiles(p.assign(path).append("\\").append(fileinfo.name), filepaths, ownname);
			}
			else
			{
				filepaths.push_back(path + "\\" + fileinfo.name);
				ownname.push_back(fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

/*
* Get all files in path
*/
void dataIO::getAllFiles()
{
	dataIO::GetAllFiles(path, filepaths, ownnames);
}

// image
/*
@param flags Flag that can take values of cv::ImreadModes.
*/
cv::Mat dataIO::readImg(int flags)
{
	string openFilename = path + filename;
	cv::Mat img = cv::imread(openFilename, flags);
	return img;
}

/*
@param img Image you want to save.
@param addName Name that will be insert into the end of file's pure name and suffix name.
@param suffix The suffix can be set, if you want to convert input image.
*/
bool dataIO::saveImg(cv::Mat img, string addName, string suffix)
{
	string pureName = filename.substr(0, filename.rfind("."));     // 获取不带后缀名的纯文件名 // rfind = reverse find
	if (suffix == "")
		suffix = filename.substr(filename.find_last_of('.'));  	  // 获取后缀名（获取filename从最后一个点的位置开始直到最后的字符）
	if (savePath == "")
		savePath = path;
	string openFilename = savePath + pureName + addName + suffix;
	return cv::imwrite(openFilename, img);
}

vector<string> dataIO::readFromDataset(std::string fileList)
{
	vector<string> v;
	fstream file(fileList);
	if (!file.is_open())
		throw std::runtime_error("Failed to read depth list");

	std::string dir;
	size_t slashIdx = fileList.rfind('/');
	slashIdx = slashIdx != std::string::npos ? slashIdx : fileList.rfind('\\');
	dir = fileList.substr(0, slashIdx);

	while (!file.eof())
	{
		std::string s, imgPath;
		std::getline(file, s);
		if (s.empty() || s[0] == '#') continue;
		std::stringstream ss;
		ss << s; // ss是一行完整的内容
		double thumb;
		ss >> thumb >> imgPath; // thumb是这一行的浮点数（即数据集的时间戳），imgPath是去掉开头浮点数的后面部分
		v.push_back(dir + '/' + imgPath); // 
	}

	return v;
}

/*
Get all the depth images with cv::IMREAD_ANYDEPTH listed in txt file located in savePath + filename.
Note: 
	If savePath is not defined, then savePath equals to path.
	Once this function is executed, changing of savePath, path or filename will not change the txt file originally located in savePath + filename.
*/
cv::Mat dataIO::getDepth()
{
	cv::Mat out;
	if (savePath == "")
		savePath = path;
	static string openFilename = path + filename;
	fileList = readFromDataset(openFilename);
	if (frameIdx < fileList.size())
	{
		cv::Mat f = cv::imread(fileList[frameIdx++], cv::IMREAD_ANYDEPTH); //对于AzureKinect, RealSense D415,D435,D435i 都是f.type()=out.type()=2（CV_16UC1）
		f.copyTo(out); // Mat转换为UMat
	}
	else
	{
		return cv::Mat();
	}
	if (out.empty())
		throw std::runtime_error("Matrix is empty");
	return out;
}

/*
Get all the images with cv::IMREAD_COLOR listed in txt file located in savePath + filename.
Note: 
	If savePath is not defined, then savePath equals to path.
	Once this function is executed, changing of savePath, path or filename will not change the txt file originally located in savePath + filename.
*/
cv::Mat dataIO::getImage()
{
	cv::Mat out;
	if (savePath == "")
		savePath = path;
	static string openFilename = path + filename;
	fileList = readFromDataset(openFilename);
	if (frameIdx < fileList.size())
	{
		cv::Mat f = cv::imread(fileList[frameIdx++], cv::IMREAD_COLOR);
		f.copyTo(out); // Mat转换为UMat
	}
	else
	{
		return cv::Mat();
	}
	if (out.empty())
		throw std::runtime_error("Matrix is empty");
	return out;
}

#endif
