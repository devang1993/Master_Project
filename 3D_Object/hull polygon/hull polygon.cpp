#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pointer;

void Load_PCDFile(cloud_pointer& cloud);


void Load_PCDFile(cloud_pointer& cloud)
{
	//==========================
	// Pointcloud Visualization
	//==========================
	// Create viewer object titled "Captured Frame"
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Captured Frame"));

	// Set background of viewer to black
	viewer->setBackgroundColor(0, 0, 0);
	// Add generated point cloud and identify with string "Cloud"
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "Cloud");
	// Default size for rendered points
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
	// Viewer Properties
	viewer->initCameraParameters();  // Camera Parameters for ease of viewing

	cout << endl;
	cout << "Press [Q] in viewer to continue. " << endl;

	viewer->spin(); // Allow user to rotate point cloud and view it

	// Note: No method to close PC visualizer, pressing Q to continue software flow only solution.

}


int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
		cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>),
		cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCDReader reader;

	reader.read("Captured_Frame1.pcd", *cloud);

	Load_PCDFile(cloud);

	// Build a filter to remove spurious NaNs
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.1);
	pass.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering has: "
		<< cloud_filtered->points.size() << " data points." << std::endl;

	Load_PCDFile(cloud_filtered);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);
	std::cerr << "PointCloud after segmentation has: "
		<< inliers->indices.size() << " inliers." << std::endl;

	// Project the model inliers
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setIndices(inliers);
	proj.setInputCloud(cloud_filtered);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
	std::cerr << "PointCloud after projection has: "
		<< cloud_projected->points.size() << " data points." << std::endl;

	Load_PCDFile(cloud_projected);

	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ConcaveHull<pcl::PointXYZRGB> chull;
	chull.setInputCloud(cloud_projected);
	chull.setAlpha(0.1);
	chull.reconstruct(*cloud_hull);

	Load_PCDFile(cloud_hull);

	std::cerr << "Concave hull has: " << cloud_hull->points.size()
		<< " data points." << std::endl;

	pcl::PCDWriter writer;
	writer.write("Captured_Frame1_hull.pcd", *cloud_hull, false);

	return (0);
}