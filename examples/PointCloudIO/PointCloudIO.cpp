#include "..\..\dataIO.h"

typedef pcl::PointXYZ PointT;

int main()
{
	dataIO<PointT> file;
	file.path = "..\\testSamples\\pointcloud\\";

	file.pcFilename = "bun045.ply";
	pcl::PointCloud<PointT> src = file.readPC();
	file.pcFilename = "bun000.ply";
	pcl::PointCloud<PointT> tgt = file.readPC();
	
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