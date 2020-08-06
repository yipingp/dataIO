# dataIO
## Introduction

A header only library for convenient coding in the field of computer vision.

### Supported features

* Data (including txt, OpenCV image, PCL point cloud) input and output

* Get files in a folder
* Read/Write RGBD dataset
* TCP/IP
* Point cloud rendering.

## Dependencies

* Linux (tested on Ubuntu 18.04
* OpenCV
* PCL (only required when PointCloudIO.hpp is included)

## Usage

### Read files from a folder

```c++
void dataIO::getAllFiles();
```

Complete version:

```c++
#include "dataIO.hpp"

int main()
{
    dataIO files;
    files.path=".";
    files.getAllFiles();

    // filename
    for (auto ownname:files.ownnames)
        cout << ownname << endl;

    // directory + filename
    for (auto filepath : files.filepaths)
        cout << filepath << endl;
}
```
### Read RGB-D dataset

Reading dataset downloaded from [here](https://vision.in.tum.de/data/datasets/rgbd-dataset) or of your own (detailed below) is both available.

```c++
template <typename T>
T dataIO::getImage<T>();
T dataIO::getDepth<T>();
```

Complete version:

``````c++
#include "dataIO.hpp"

int main()
{
    dataIO colorIO, depthIO;
    colorIO.path = "RGBD_dataset"; // the folder of dataset
    colorIO.filename = "rgb.txt";
    depthIO.path = colorIO.path;
    depthIO.filename = "depth.txt";
    
    while(true)
    {
        cv::Mat color = colorIO.getImage<Mat>();
        cv::Mat depth = depthIO.getDepth<Mat>();
    }
}
``````

### Write RGB-D dataset

For users of RGB-D cameras, we create a dataset with the following structure under ```dataset_dir```.

RGBD_dataset

├── depth

│   ├── 1.2.png

│   ├── 2.2.png

│   └── ...

├── depth.txt

├── rgb

│   ├── 1.1.png

│   ├── 2.1.png

│   └── ...

└── rgb.txt


```c++
#include "dataIO.hpp"

/************Parameters************/
string dataset_dir = ".";
string dataset_folder = "RGBD_dataset";

int main()
{
    dataIO files;
    files.path = dataset_dir;
    vector<string> colorNames, depthNames; // Names of color images and depth images

    for (int i = 0; i < 10; i++)
    {
        files.save_RGBD_dataset(dataset_folder, colorNames, depthNames); // First we create a folder named RGBD_dataset under files.path

        static int num_cnt = 1;
        stringstream file_cnt;
        file_cnt << num_cnt;
        num_cnt++;
        // ".1" is used to name color images, the first color image is named "1.1.png"
        // ".2" is used to name depth images, the first color image is named "1.2.png"
        string color_path = files.joinPath(files.path, dataset_folder) + "/" + files.joinPath("rgb", file_cnt.str()) + ".1.png";
        string depth_path = files.joinPath(files.path, dataset_folder) + "/" + files.joinPath("depth", file_cnt.str()) + ".2.png";
        colorNames.push_back(color_path);
        depthNames.push_back(depth_path);

        cv::Mat color;
        cv::Mat depth;
        /*
        image caputring..
        complemented by your own depending on your camera
        */
        cv::imwrite(color_path, color); // Write captured color image and depth image
        cv::imwrite(depth_path, depth);
    }
    files.save_RGBD_dataset("RGBD_dataset", colorNames, depthNames); // write rgb.txt and depth.txt
}
```

### TCP/IP Server

```C++
#include "dataIO.hpp"

int main()
{
    dataIO files;
    files.tcpipInit(6666); // SERVER_PORT is 6666
    
    // After connected
    string str = "message";
    files.tcpipSend(str); // Send message to client
}
```

### IO and visualization of point clouds

```c++
template <class PointType>
void PointCloudIO::PCRender(pcl::PointCloud<PointType> cloudView1, pcl::PointCloud<PointType> cloudView2 = pcl::PointCloud<PointType>(), pcl::PointCloud<PointType> cloudView3 = pcl::PointCloud<PointType>(), pcl::PointCloud<PointType> cloudView4 = pcl::PointCloud<PointType>())
```

Complete version:

```c++
#included "PointCloudIO.hpp"

typedef pcl::PointXYZRGB PointT;

int main()
{
	PointCloudIO<PointT> file;
	file.path = "cloud_files";

	file.filename = "cloud1.ply"; pcl::PointCloud<PointT> cloud1 = file.readPC();
	file.filename = "cloud2.ply"; pcl::PointCloud<PointT> cloud2 = file.readPC();

	// optinal settings for PC rendering
	/*
	file.set_txt_content("cloud1", "cloud2"); // set text content respectively
	file.set_txt_rgb(pcl::PointXYZ(0.7, 0.7, 0.7), pcl::PointXYZ(0.7, 0.7, 0.7)); // set text color respectively
	file.rgb_bkg = pcl::PointXYZ(0.3, 0.3, 0.3); // set backgound color
	file.coordinateScale = 0.05; // set scale of coordinate
	*/

	file.PCrender(cloud1, cloud2); // 4 point clouds for visualization at most

	return 0;
}
```
