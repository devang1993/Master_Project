#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

int main(int argc, char** argv)
{
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
	if (pcl::io::loadPCDFile <pcl::PointXYZ>("min_cut_segmentation_tutorial.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::MinCutSegmentation<pcl::PointXYZ> seg;
	seg.setInputCloud(cloud);
	seg.setIndices(indices);

	pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ point;
	point.x = 68.97;
	point.y = -18.55;
	point.z = 0.57;
	foreground_points->points.push_back(point);
	seg.setForegroundPoints(foreground_points);

	seg.setSigma(0.25);
	seg.setRadius(3.0433856);
	seg.setNumberOfNeighbours(14);
	seg.setSourceWeight(0.8);

	std::vector <pcl::PointIndices> clusters;
	seg.extract(clusters);

	std::cout << "Maximum flow is " << seg.getMaxFlow() << std::endl;

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud();
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->spin();
	while (!viewer->wasStopped())
	{
	}

	return (0);
}