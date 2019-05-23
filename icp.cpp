/* TILT PROJECT */

#include <iostream>
#include <algorithm> 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

 // Intel Realsense Headers
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

// Euclidean Clusture Extraction
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


using namespace std;

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;

// Prototypes
void Load_PCDFile(cloud_pointer& cloud);
bool userInput(void);
int PCL_ICP(cloud_pointer& cloud1, cloud_pointer& cloud2);
cloud_pointer cluster(cloud_pointer& cloud);
// Global Variables
int i = 1; // Index for incremental file name

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================


std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
	// Get Width and Height coordinates of texture
	int width = texture.get_width();  // Frame width in pixels
	int height = texture.get_height(); // Frame height in pixels

	// Normals to Texture Coordinates conversion
	int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
	int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

	int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
	int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
	int Text_Index = (bytes + strides);

	const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

	// RGB components to save in tuple
	int NT1 = New_Texture[Text_Index];
	int NT2 = New_Texture[Text_Index + 1];
	int NT3 = New_Texture[Text_Index + 2];

	return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 

cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color) {

	// Object Declaration (Point Cloud)
	cloud_pointer cloud(new point_cloud);

	// Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
	std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

	//================================
	// PCL Cloud Object Configuration
	//================================
	// Convert data captured from Realsense camera to Point Cloud
	auto sp = points.get_profile().as<rs2::video_stream_profile>();

	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto Texture_Coord = points.get_texture_coordinates();
	auto Vertex = points.get_vertices();

	// Iterating through all points and setting XYZ coordinates
	// and RGB values
	for (int i = 0; i < points.size(); i++)
	{
		//===================================
		// Mapping Depth Coordinates
		// - Depth data stored as XYZ values
		//===================================
		cloud->points[i].x = Vertex[i].x;
		cloud->points[i].y = Vertex[i].y;
		cloud->points[i].z = Vertex[i].z;

		// Obtain color texture for specific point
		RGB_Color = RGB_Texture(color, Texture_Coord[i]);

		// Mapping Color (BGR due to Camera Model)
		cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
		cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
		cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

	}

	return cloud; // PCL RGB Point Cloud generated
}

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

//========================================
// userInput
// - Prompts user for a char to 
// test for decision making.
// [y|Y] - Capture frame and save as .pcd
// [n|N] - Exit program
//========================================

bool userInput(void) {

	bool setLoopFlag;
	bool inputCheck = false;
	char takeFrame; // Utilize to trigger frame capture from key press ('t')

	do
	{
		// Prompt User to execute frame capture algorithm
		cout << endl;
		cout << "Generate a Point Cloud? [y/n] ";
		cin >> takeFrame;
		cout << endl;
		// Condition [Y] - Capture frame, store in PCL object and display
		if (takeFrame == 'y' || takeFrame == 'Y') {
			setLoopFlag = true;
			inputCheck = true;
			takeFrame = 0;
		}
		// Condition [N] - Exit Loop and close program
		else if (takeFrame == 'n' || takeFrame == 'N') {
			setLoopFlag = false;
			inputCheck = true;
			takeFrame = 0;
		}
		// Invalid Input, prompt user again.
		else {
			inputCheck = false;
			cout << "Invalid Input." << endl;
			takeFrame = 0;
		}
	} while (inputCheck == false);

	return setLoopFlag;
}

int PCL_ICP(cloud_pointer& cloud1, cloud_pointer& cloud2)
{
	// PCL ICP for computation of the transformation matrix

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cout << "\nstarting icp" << endl;
	
	std::cout << "PointCloud1 before filtering has: " << cloud1->points.size() << " data points." << std::endl; //*
	std::cout << "PointCloud2 before filtering has: " << cloud2->points.size() << " data points." << std::endl; //*

	pcl::VoxelGrid<RGB_Cloud> grid;
	grid.setLeafSize(0.01f, 0.01f, 0.01f);
	grid.setInputCloud(cloud1);
	grid.filter(*cloud1);
	Load_PCDFile(cloud1);
	grid.setInputCloud(cloud2);
	grid.filter(*cloud2);
	Load_PCDFile(cloud2);
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

	// Euclidean Cluster Extraction
	cluster(cloud1);
	cluster(cloud2);

	cout << "\nset Maximum number of iterations:   ";
	cin >> i;
	icp.setMaximumIterations(i);
	icp.setInputSource(cloud2);
	icp.setInputTarget(cloud1);
	icp.setTransformationRotationEpsilon(0.95);
	icp.setMaxCorrespondenceDistance(0.5);
	icp.align(*cloud);
	Load_PCDFile(cloud);
	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\n Final transformation matrix is : \n" << icp.getFinalTransformation();
		return EXIT_SUCCESS;
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		std::cout << "\nPress Enter to exit Program... ";
		return (-1);
	}

}

//============================
//Euclidean Cluster Extraxtion
//============================

cloud_pointer cluster(cloud_pointer& cloud)
{

	std::cout << "PointCloud after filtering has: " << cloud->points.size() << " data points." << std::endl; //*

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	int i = 0, nr_points = (int)cloud->points.size();
	while (cloud->points.size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud = *cloud_f;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(250);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_cluster, false); //*
		j++;
	}
	Load_PCDFile(cloud);
	return cloud;
}
//==============
// Main function
//==============

int main()
{
	//======================
	// Variable Declaration
	//======================
	bool captureLoop = true; // Loop control for generating point clouds
	std::vector<cloud_pointer> p_cloud;

	//====================
	// Object Declaration
	//====================
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> openCloud;

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;

	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;

	//======================
	// Stream configuration
	//======================
	cfg.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_INFRARED, 424, 240, RS2_FORMAT_Y8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 30);

	rs2::pipeline_profile selection = pipe.start(cfg);

	rs2::device selected_device = selection.get_device();
	auto depth_sensor = selected_device.first<rs2::depth_sensor>();

	if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
	{
		depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
		depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
	}
	if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
	{
		// Query min and max values:
		auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
		depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
		depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
	}

	// Begin Stream with default configs

	// Loop and take frame captures upon user input
	while (captureLoop == true && i < 3) {

		// Set loop flag based on user input
		captureLoop = userInput();
		if (captureLoop == false) { break; }


		// Wait for frames from the camera to settle
		for (int i = 0; i < 30; i++) {
			auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
		}

		// Capture a single frame and obtain depth + RGB values from it    
		auto frames = pipe.wait_for_frames();
		auto depth = frames.get_depth_frame();
		auto RGB = frames.get_color_frame();

		// Map Color texture to each point
		pc.map_to(RGB);

		// Generate Point Cloud
		auto points = pc.calculate(depth);

		// Convert generated Point Cloud to PCL Formatting
		cloud_pointer cloud = PCL_Conversion(points, RGB);

		//========================================
		// Filter PointCloud (PassThrough Method)
		//========================================
		pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
		Cloud_Filter.setInputCloud(cloud);           // Input generated cloud to filter
		Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
		Cloud_Filter.setFilterLimits(0.0, 2.0);      // Set accepted interval values
		Cloud_Filter.filter(*cloud);              // Filtered Cloud Outputted

		p_cloud.push_back(cloud);

		cout << " Pointcloud successfully generated. " << endl;

		//Load generated PCD file for viewing
		Load_PCDFile(p_cloud.at(i - 1));
		i++; // Increment File Name

	}//End-while

	PCL_ICP(p_cloud.at(0), p_cloud.at(1)); // Run ICP on the pointclouds

	system("pause");

	return EXIT_SUCCESS;
}