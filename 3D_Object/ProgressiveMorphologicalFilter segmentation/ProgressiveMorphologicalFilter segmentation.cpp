#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/cloud_viewer.h>

#include <string>
#include <pcl/filters/voxel_grid.h>


using namespace std;

void view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->resetCamera();
	viewer->spin();
}

int
main(int argc, char** argv)
{
	string file;
	cout << "file name\t";
	cin >> file;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointIndicesPtr ground(new pcl::PointIndices);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZRGB>(file, *cloud);
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
	grid.setLeafSize(0.05f, 0.05f, 0.05f);
	grid.setInputCloud(cloud);
	grid.filter(*cloud);

	view(cloud);

	// Create the filtering object
	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZRGB> pmf;
	pmf.setInputCloud(cloud);
	pmf.setMaxWindowSize(20);
	pmf.setSlope(1.0f);
	pmf.setInitialDistance(0.5f);
	pmf.setMaxDistance(3.0f);
	pmf.extract(ground->indices);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(ground);
	extract.filter(*cloud_filtered);
	std::cerr << "Ground cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGB>(file + "_ground.pcd", *cloud_filtered, false);

	view(cloud_filtered);

	// Extract non-ground returns
	extract.setNegative(true);
	extract.filter(*cloud_filtered);
	std::cerr << "Object cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	writer.write<pcl::PointXYZRGB>(file + "_object.pcd", *cloud_filtered, false);

	view(cloud_filtered);

	return (0);
}