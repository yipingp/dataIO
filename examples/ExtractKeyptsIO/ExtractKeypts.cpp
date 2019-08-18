#include "..\..\dataIO.h"

int main()
{
	dataIO<> file;
	file.path= "..\\testSamples\\ExtractKeypts\\";

	file.filename = "1.jpg";
	cv::Mat img = file.readImg();

	// keyPts = extract(img)   // Extracting keypoints from img...
	cv::Point2f pt(1, 2);
	vector<cv::Point2f> keyPts;
	keyPts.push_back(pt);
	keyPts.push_back(pt);

	file.save2dPts(keyPts, "_keyPts");

	return 0;

}