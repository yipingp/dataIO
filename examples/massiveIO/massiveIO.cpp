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