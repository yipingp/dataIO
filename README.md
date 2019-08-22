# dataIO
Convenient data(including txt, OpenCV image, PCL point cloud) input and output, and convenient point cloud rendering.

Using dataIO class, you are able to instantly organize input and output files just by setting a main path. 

The features especially includes:  
    1. ***Conveniently adding names after names of input files:*** for the purpose of distinguishing, for instance, "test.jpg" to "test_processed.jpg", "cloud.ply" to "cloud_aligned.ply", etc.  
    2. ***Point cloud rendering: based on PCL(Point Cloud Library):*** using member function *PCrender(pointcloud1,...,pointcloud4)* to visualize up to 4 pointclouds without manually adjusting viewport. Additionally, parameters like color of backgournd, color of text, content of text, scale of coordinate are all adjustable as you wish.  
    3. ***Massive data IO:*** getting all files in main path set by member variable *path* using member function *getAllFiles()*.  
    4. For more features, please check the **examples**.
    
    
# CV工程常用数据IO类（C++实现）)
# 简介

通过C++类模板来完成CV工程中常用数据（txt、图像、点云）的**快速读取**和带有添加标记的**快速保存**（如工程实践中，通常需要将保存的文件另外命名以作区别，如"test.jpg"保存为"test_saved.jpg"，"pointcloud.ply"在经过配准后保存为pointcloud_aligned.ply"，这仅仅需要使用类模板的成员函数saveImg(图像变量,"_saved")即可）。

此外，该类模板还可以方便地完成**读取txt文件中的二维/三维坐标点**、**读取文件夹中所有文件**、**图像IO**、**点云IO和显示**。同时，**复杂文件命名情况的IO**也可以很方便地解决。

# 读取txt文件中二维/三维坐标点

```
#include "..\..\dataIO.h"

int main()
{
	dataIO<> files;
	files.path = "..\\testSamples\\txt\\";

	// 2d points
	files.filename = "2d_Points.txt";
	vector<cv::Point2f> Pts_2d = files.read2dPts();
	cout << Pts_2d << endl << endl;
	files.save2dPts(Pts_2d,"_saved");

	// 3d points
	files.filename = "3d_Points.txt";
	vector<cv::Point3f> Pts_3d = files.read3dPts();
	cout << Pts_3d;
	files.save3dPts(Pts_3d, "_saved");

	return 0;
}
```
# 读取文件夹中所有文件

```
#include "..\..\dataIO.h"

int main()
{
	dataIO<> file;
	file.path = "..\\testSamples\\img\\";
	file.savePath = file.path + "res\\";  // .savePath will remain as defulat (equals to path) if not initialized. 

	// Get all files in .path, and processing them one by one.
	file.getAllFiles();
	for (int i = 0; i < file.ownnames.size(); i++)
	{
		file.filename = file.ownnames[i];
		cv::Mat img = file.readImg();

		// processing current img...

		file.saveImg(img, "_processed");
	}

	return 0;
}
```
# 图像IO

```
#include "..\..\dataIO.h"

int main()
{
	dataIO<> file;
	file.path = "..\\testSamples\\img\\";

	file.filename = "img1.jpg";
	cv::Mat img = file.readImg();

	// processing img...

	file.saveImg(img, "_processed");

	return 0;
}
```
# 点云IO和显示
&emsp;&emsp;仅使用一行代码可以完成pcl::visualization::PCLVisualizer类的实例化，最多支持4组点云的自动viewport设置
```
file.PCrender(src, tgt, res, cloudEmpty);
```

&emsp;&emsp;完整代码如下。
```
#include "..\..\dataIO.h"

typedef pcl::PointXYZ PointT;

int main()
{
	dataIO<PointT> file;
	file.path = "..\\testSamples\\pointcloud\\";

	file.filename = "bun000.ply";
	pcl::PointCloud<PointT> tgt = file.readPC();
	file.filename = "bun045.ply";
	pcl::PointCloud<PointT> src = file.readPC();
	
	pcl::PointCloud<PointT> res;
	pcl::PointCloud<PointT> cloudEmpty;

	// ICP aligning...
	res = src + tgt; // Simulating ICP result


	// optinal settings for PC rendering
	/*
	file.set_txt_content("src_cloud", "tgt_cloud", "res_cloud");												// set text content
	file.set_txt_rgb(pcl::PointXYZ(0.7, 0.7, 0.7), pcl::PointXYZ(0.7, 0.7, 0.7), pcl::PointXYZ(0.7, 0.7, 0.7)); // set text color
	file.rgb_bkg = pcl::PointXYZ(0.3, 0.3, 0.3);																// set backgound color
	file.coordinateScale = 0.05;																				// set scale of coordinate
	*/

	file.PCrender(src, tgt, res, cloudEmpty);
	file.savePC(res, "_aligned");

	return 0;
}
```

# 复杂文件命名情况的IO
&emsp;&emsp;这里，我们进行的是基于RGB-D和存于txt文件中RGB图上的二维特征点来进行特征点云的获取。

&emsp;&emsp;当前需要进行读入的文件有：


 - **深度图**"depth3.png"
 ```
files.filename = "depth3.png";
cv::Mat depth = files.readImg();  // depth image
```
 - **彩色图**"color3.png"
 ```
 files.filename = "color3.png";
cv::Mat img = files.readImg();    // color image
```
 - RGB图上的**二维特征点坐标**"color3_2D_keypoints.txt"
```
// 2d key points 
// add "_2D_keypoints" after "color3" as txt file's name is color3_2D_keypoints
vector<cv::Point2f> keypts = files.read2dPts("_2D_keypoints"); 
```

&emsp;&emsp;需要进行输出的有：

- 存于txt文件的**三维特征点云坐标**（命名为"color3_keypoint_cloud.txt"）
```
files.save3dPts(keypts_3d, "_keypoint_cloud");
```

- **三维特征点云**（命名为"color3_keypoint_cloud.ply"）
```
// Force setting suffix to ".ply" as filename is currently "color3.png".
files.savePC(cloud, "_keypoint_cloud", ".ply"); 
```

&emsp;&emsp;具体代码如下。
```
#include "..\..\dataIO.h"

typedef pcl::PointXYZ PointT;

// Generating 3d key point cloud (txt and pcl point cloud) from RGB-D image and 2d key points.
// All you have to change is "depth3.png" and "color3.png" when input images vary.
int main()
{
	dataIO<PointT> files;
	files.path = "..\\testSamples\\keyPointcloud\\";

	files.filename = "depth3.png";
	cv::Mat depth = files.readImg();  // depth image

	files.filename = "color3.png";
	cv::Mat img = files.readImg();    // color image
	vector<cv::Point2f> keypts = files.read2dPts("_2D_keypoints"); // 2d key points // add "_2D_keypoints" after "color3" as txt file's name is color3_2D_keypoints


	// Generating 3d key point cloud (txt and pcl point cloud)...
	vector<cv::Point3f> keypts_3d;
	pcl::PointCloud<PointT> cloud; cloud.width = 1; cloud.height = 1; cloud.is_dense = false; cloud.points.resize(cloud.width * cloud.height);cloud.points[0].x = 1024 * rand() / (RAND_MAX + 1.0f);cloud.points[0].y = 1024 * rand() / (RAND_MAX + 1.0f);cloud.points[0].z = 1024 * rand() / (RAND_MAX + 1.0f);


	// Saving...
	files.save3dPts(keypts_3d, "_keypoint_cloud");
	files.savePC(cloud, "_keypoint_cloud", ".ply"); // Force setting suffix to ".ply" as filename is currently "color3.png".

	return 0;

}
```


