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