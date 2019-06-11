#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int
main(int argc, char *argv[]) {

	int m;
	char again;
	do {
		cout << "method 1 or 2" << endl;
		cin >> m;

		PointCloudT::Ptr cloud(new PointCloudT);

		// Visualization
		pcl::visualization::PCLVisualizer viewer("PCL visualizer");

		// Fill in the cloud data
		cloud->width = 500;
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);

		for (size_t i = 0; i < cloud->points.size(); ++i) {
			cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);

			//Method #1 - Random color for each point
			if (m == 1) {
				cloud->points[i].r = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
				cloud->points[i].g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
				cloud->points[i].b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
			}
		}
		if (m == 1)
			viewer.addPointCloud(cloud, "cloud"); // Method #1

			//Method #2 - Global color for all points (with this method all points have the same color)
		if (m == 2) {
			pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler(cloud, 255, 200, 255);
			viewer.addPointCloud(cloud, cloud_color_handler, "cloud");
		}

		viewer.addCoordinateSystem(1.0, "axis", 0);
		viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);	// Setting background to a dark grey
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

		while (!viewer.wasStopped()) {
			viewer.spinOnce();
		}
		cout << "again? Y/n" << endl;
		cin >> again;
	} while (again == 'y' || again == 'y');
	return (0);
}