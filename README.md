# dataIO
Convenient data(including txt, OpenCV image, PCL point cloud) input and output, and convenient point cloud rendering.

Using dataIO class, you are able to instantly organize input and output files just by setting a main path. 
The features especially includes:
    1. ***Conveniently adding names after names of input files:*** for the purpose of distinguishing, for instance, "test.jpg" to "test_processed.jpg", "cloud.ply" to "cloud_aligned.ply", etc.
    2. ***Point cloud rendering: based on PCL(Point Cloud Library):*** using member function *PCrender(pointcloud1,...,pointcloud4)* to visualize up to 4 pointclouds without manually adjusting viewport. Additionally, parameters like color of backgournd, color of text, content of text, scale of coordinate are all adjustable as you wish.
    3. ***Massive data IO:*** getting all files in main path set by member variable *path* using member function *getAllFiles()*.
    4. For more features, please check the **examples**.
