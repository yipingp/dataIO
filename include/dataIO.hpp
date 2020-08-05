#pragma once

#include <opencv2/opencv.hpp>

#include <vector>
#include <dirent.h> // GetAllFiles()中用到
#include <sys/types.h>
#include <sys/stat.h>
#include <io.h>

// TCP/IP
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;

class dataIO
{
public:
	// 路径
	string path; // 文件路径
	string savePath; // 存储路径，如果没有设置，则保持和path相同
	string joinPath(string dir, string file); // 将dir中所有的\\换成/，并判断最后有没有/，没有的话补上

	// txt
	string filename; // 文件名，不管是读取还是保存都需要设置（除了save_RGBD_dataset()不需要外）
	int savePts(vector<cv::Point2f> &Data, string addName = "");
	int savePts(vector<cv::Point3f> &Data, string addName = "");
	vector<cv::Point2f> read2dPts(string addName = "");
	vector<cv::Point3f> read3dPts(string addName = "");
	int save_RGBD_dataset(string folderOfDataset, vector<string> filenamesOfRGBs, vector<string> filenamesOfDepths);

	// Get all files in the path
	vector<string> filepaths; // directory + file names
	vector<string> ownnames; // file names
	void getAllFiles();

	// image
	int frameIdx = 0;
	vector<string> fileList;
	cv::Mat readImg(int flags = cv::IMREAD_UNCHANGED);
	bool saveImg(cv::Mat img, string addName = "", string suffix = "");
	// (获取RGBD dataset的RGB图和深度图)
	template <typename T>
	T getImage();
	template <typename T>
	T getDepth();

	// communication
	// (TCPIP)
	void tcpipInit(int SERVER_PORT = 6666); // 建立服务器，监听SERVER_PORT端口
	void tcpipSend(string str);

private:	
	// files
	void GetAllFiles(string path, vector<string> &filepaths, vector<string> &ownname);
	// image
	vector<string> readFromDataset(std::string fileList);
	// communication
	// (TCPIP)
	int client_;
};

// 路径
inline string dataIO::joinPath(string dir, string file)
{
	string res;
	while (!(dir.find("\\") == dir.npos))
		dir = dir.replace(dir.find("\\"), 1, "/");
	// 最后一位是dir.length - 1，因为从0开始
	if (dir.rfind("/") != dir.length() - 1) // 路径dir最后没带/的情况
		res = dir + "/" + file;
	else
		res = dir + file;
	return res;
}

// txt
/*
@param Data 2d points data you want to save.
@param addName Name that will be insert into the end of file's pure name and suffix name.
*/

inline int dataIO::savePts(vector<cv::Point2f> &Data, string addName)
{
	string pureName = filename.substr(0, filename.rfind(".")); // 获取不带后缀名的纯文件名 // rfind = reverse find
	string suffix = ".txt";
	if (savePath == "")
		savePath = path;
	string openFilename = joinPath(savePath, pureName);
	openFilename = openFilename + addName + suffix;

	// 以只读模式创建新文件
	fstream openTXT(openFilename, fstream::out);
	openTXT.close();

	fstream writeTXT;
	writeTXT.open(openFilename);
	assert(writeTXT.is_open()); //若失败,则输出错误消息,并终止程序运行
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
inline int dataIO::savePts(vector<cv::Point3f> &Data, string addName)
{
	string pureName = filename.substr(0, filename.rfind("."));
	string suffix = ".txt";
	if (savePath == "")
		savePath = path;
	string openFilename = joinPath(savePath, pureName);
	openFilename = openFilename + addName + suffix;

	fstream openTXT(openFilename, fstream::out);
	openTXT.close();

	fstream writeTXT;
	writeTXT.open(openFilename);
	assert(writeTXT.is_open()); //若失败,则输出错误消息,并终止程序运行
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
Create a folder named folderOfDataset under the member variable path or savePath, in which consists of depth.txt, rgb.txt, as well as folders named depth and rgb repectively.
@param folderOfDataset Folder's name of the dataset to be created.
@param filenamesOfRGBs file paths of RGB images (Use absolute path here, relative path recorded in rgb.txt will be truncated automatically.)
@param filenamesOfDepths file paths of depth images (Use absolute path here, relative path recorded in depth.txt will be truncated automatically.)
*/
inline int dataIO::save_RGBD_dataset(string folderOfDataset, vector<string> filenamesOfRGBs, vector<string> filenamesOfDepths)
{
	if (savePath == "")
		savePath = path;
	string openFilename = joinPath(savePath, folderOfDataset);

	// 创建文件夹
	string temp = openFilename;
	mkdir(temp.data(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); // 在路径savePath下创建folderOfDataset
	temp = openFilename + "/depth";
	mkdir(temp.data(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	temp = openFilename + "/rgb";
	mkdir(temp.data(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	/****写到folderOfDataset文件夹下的depth.txt*****/
	fstream openDepthTXT(openFilename + "/depth.txt", fstream::out);
	openDepthTXT.close();

	fstream writeDepthTXT;
	writeDepthTXT.open(openFilename + "/depth.txt");
	assert(writeDepthTXT.is_open()); //若失败,则输出错误消息,并终止程序运行
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
		writeDepthTXT << filenamesOfDepths[i].substr(idx1, idx2 - idx1) + " " + filenamesOfDepths[i].substr(idx3) << endl;
	}
	writeDepthTXT.close();

	/****写到folderOfDataset文件夹下的rgb.txt*****/
	fstream openRgbTXT(openFilename + "/rgb.txt", fstream::out);
	openRgbTXT.close();

	fstream writeRgbTXT;
	writeRgbTXT.open(openFilename + "/rgb.txt");
	assert(writeRgbTXT.is_open()); //若失败,则输出错误消息,并终止程序运行
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
inline vector<cv::Point2f> dataIO::read2dPts(string addName)
{
	string pureName = filename.substr(0, filename.rfind("."));
	string suffix = ".txt";
	string openFilename = joinPath(savePath, pureName);
	openFilename = openFilename + addName + suffix;
	vector<cv::Point2f> pts;
	ifstream readTXT;
	readTXT.open(openFilename.data()); //将文件流对象与文件连接起来
	assert(readTXT.is_open());
	cv::Point2f point;

	while (!readTXT.eof())
	{
		readTXT >> point.x >> point.y;
		if(readTXT.good())
			pts.push_back(point);
	}
	// pts.erase(pts.begin() + pts.size() - 1); // 去除重复读取的最后一行
	readTXT.close();
	return pts;
}

/*
@param addName Name that will be insert into the end of file's pure name and suffix name.
*/
inline vector<cv::Point3f> dataIO::read3dPts(string addName)
{
	string pureName = filename.substr(0, filename.rfind("."));
	string suffix = ".txt";
	string openFilename = joinPath(savePath, pureName);
	openFilename = openFilename + addName + suffix;
	vector<cv::Point3f> pts;
	ifstream readTXT;
	readTXT.open(openFilename.data()); //将文件流对象与文件连接起来
	assert(readTXT.is_open());
	cv::Point3f point;

	while (!readTXT.eof())
	{
		readTXT >> point.x >> point.y >> point.z;
		if(readTXT.good())
			pts.push_back(point);
	}
	// pts.erase(pts.begin() + pts.size() - 1); // 去除重复读取的最后一行
	readTXT.close();
	return pts;
}

//files
/* 
* https://www.cnblogs.com/yuehouse/p/10159358.html
* path为文件夹路径
* filepaths存储文件的路径及名称
* ownname只存储文件的名称
*/
inline void dataIO::GetAllFiles(string path, vector<string> &filepaths, vector<string> &ownname)
{
	DIR *pDir;
	struct dirent *ptr;
	if (!(pDir = opendir(path.c_str())))
	{
		cout << "Folder doesn't Exist!" << endl;
		return;
	}
	while ((ptr = readdir(pDir)) != 0)
	{
		if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
		{
			ownname.push_back(ptr->d_name);
		}
	}
	closedir(pDir);
	for (int i = 0; i < ownname.size(); i++)
		filepaths.push_back(path + "/" + ownname[i]);
}

/*
* Get all files in the path
*/
inline void dataIO::getAllFiles()
{
	GetAllFiles(path, filepaths, ownnames);
}

// image
/*
@param flags Flag that can take values of cv::ImreadModes.
*/
inline cv::Mat dataIO::readImg(int flags)
{
	string openFilename = joinPath(path, filename);
	cv::Mat img = cv::imread(openFilename, flags);
	return img;
}

/*
@param img Image you want to save.
@param addName Name that will be insert into the end of file's pure name and suffix name.
@param suffix The suffix can be set, if you want to convert input image.
*/
inline bool dataIO::saveImg(cv::Mat img, string addName, string suffix)
{
	string pureName = filename.substr(0, filename.rfind(".")); // 获取不带后缀名的纯文件名 // rfind = reverse find
	if (suffix == "")
		suffix = filename.substr(filename.find_last_of('.')); // 获取后缀名（获取filename从最后一个点的位置开始直到最后的字符）
	if (savePath == "")
		savePath = path;
	string openFilename = joinPath(savePath, pureName);
	openFilename = openFilename + addName + suffix;
	return cv::imwrite(openFilename, img);
}

inline vector<string> dataIO::readFromDataset(std::string fileList)
{
	vector<string> v;
	fstream file(fileList);
	if (!file.is_open())
		throw std::runtime_error("Failed to read file list");

	std::string dir;
	size_t slashIdx = fileList.rfind('/');
	slashIdx = slashIdx != std::string::npos ? slashIdx : fileList.rfind('\\');
	dir = fileList.substr(0, slashIdx);

	while (!file.eof())
	{
		std::string s, imgPath;
		std::getline(file, s);
		if (s.empty() || s[0] == '#')
			continue;
		std::stringstream ss;
		ss << s; // ss是一行完整的内容
		double thumb;
		ss >> thumb >> imgPath;			  // thumb是这一行的浮点数（即数据集的时间戳），imgPath是去掉开头浮点数的后面部分
		v.push_back(dir + '/' + imgPath); //
	}

	return v;
}

/*
Get all the depth images with cv::IMREAD_ANYDEPTH listed in txt file located in savePath + filename.
*/
template <typename T>
inline T dataIO::getDepth()
{
	T out;
	static string openFilename = joinPath(path, filename);
	fileList = readFromDataset(openFilename);
	if (frameIdx < fileList.size())
	{
		cv::Mat f = cv::imread(fileList[frameIdx++], cv::IMREAD_ANYDEPTH); //对于AzureKinect, RealSense D415,D435,D435i 都是f.type()=out.type()=2（CV_16UC1）
		f.copyTo(out);													   // Mat转换为UMat
	}
	else
	{
		return T();
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
template <typename T>
inline T dataIO::getImage()
{
	T out;
	if (savePath == "")
		savePath = path;
	static string openFilename = joinPath(path, filename);

	fileList = readFromDataset(openFilename);
	if (frameIdx < fileList.size())
	{
		cv::Mat f = cv::imread(fileList[frameIdx++], cv::IMREAD_COLOR);
		f.copyTo(out); // Mat转换为UMat
	}
	else
	{
		return T();
	}
	if (out.empty())
		throw std::runtime_error("Matrix is empty");
	return out;
}

inline void dataIO::tcpipInit(int SERVER_PORT)
{
    //调用socket函数返回的文件描述符
    int serverSocket;
    //声明两个套接字sockaddr_in结构体变量，分别表示客户端和服务器
    struct sockaddr_in server_addr;
    struct sockaddr_in clientAddr;
    int addr_len = sizeof(clientAddr);
    int iDataNum;
    //socket函数，失败返回-1
    //int socket(int domain, int type, int protocol);
    //第一个参数表示使用的地址类型，一般都是ipv4，AF_INET
    //第二个参数表示套接字类型：tcp：面向连接的稳定数据传输SOCK_STREAM
    //第三个参数设置为0
    if ((serverSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("socket");
        return;
    }
    bzero(&server_addr, sizeof(server_addr));

    //初始化服务器端的套接字，并用htons和htonl将端口和地址转成网络字节序
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    //ip可是是本服务器的ip，也可以用宏INADDR_ANY代替，代表0.0.0.0，表明所有地址
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    //对于bind，accept之类的函数，里面套接字参数都是需要强制转换成(struct sockaddr *)
    //bind三个参数：服务器端的套接字的文件描述符，
    if (bind(serverSocket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("connect");
        return;
    }

    //设置服务器上的socket为监听状态
    if (listen(serverSocket, 5) < 0)
    {
        perror("listen");
        return;
    }

    printf("Listening Port: %d\n", SERVER_PORT);
    // 调用accept函数后，会进入阻塞状态
    // accept返回一个套接字的文件描述符，这样服务器端便有两个套接字的文件描述符，
    // serverSocket和client。
    // serverSocket仍然继续在监听状态，client则负责接收和发送数据
    // clientAddr是一个传出参数，accept返回时，传出客户端的地址和端口号
    // addr_len是一个传入-传出参数，传入的是调用者提供的缓冲区的clientAddr的长度，以避免缓冲区溢出。
    // 传出的是客户端地址结构体的实际长度。
    // 出错返回-1
    client_ = accept(serverSocket, (struct sockaddr *)&clientAddr, (socklen_t *)&addr_len);
    printf("TCP Server is waitting for meassage...\n");
    //inet_ntoa ip地址转换函数，将网络字节序IP转换为点分十进制IP
    //表达式：char *inet_ntoa (struct in_addr);
    printf("IP is %s\n", inet_ntoa(clientAddr.sin_addr));
    printf("Port is %d\n", htons(clientAddr.sin_port));
}

inline void dataIO::tcpipSend(string str)
{
	char buffer_[200];
	strcpy(buffer_, str.c_str());
	send(client_, buffer_, strlen(buffer_), 0);
}
