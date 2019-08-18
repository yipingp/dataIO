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