/* TILT PROJECT */

#include <iostream>
#include <algorithm> 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>
#include <stdlib.h>

// Intel Realsense Headers
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>

// PCL Headers
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


using namespace std;

typedef pcl::PointXYZ XYZ_Cloud;
typedef pcl::PointCloud<XYZ_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;

// Prototypes
bool userInput(void);
cloud_pointer voxelleaf(cloud_pointer& cloud);
int PCL_ICP(cloud_pointer& cloud1, cloud_pointer& cloud2);

void Load_PCDFile(cloud_pointer& cloud); //added at 17.33 14.06

// Global Variables
int i = 1; // Index for incremental file name

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
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "Cloud");
	// Default size for rendered points
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
	// Viewer Properties
	viewer->initCameraParameters();  // Camera Parameters for ease of viewing

	cout << endl;
	cout << "Press [Q] in viewer to continue. " << endl;

	viewer->spin(); // Allow user to rotate point cloud and view it

	// Note: No method to close PC visualizer, pressing Q to continue software flow only solution.

}
//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 

cloud_pointer PCL_Conversion(const rs2::points& points) {

	// Object Declaration (Point Cloud)
	cloud_pointer cloud(new point_cloud);

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
	}

	return cloud; // PCL XYZ Point Cloud generated
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
			cout << "Exiting program..." << endl;
			Sleep(500);
			exit(0);
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

cloud_pointer voxelleaf(cloud_pointer& cloud) {
	pcl::VoxelGrid<XYZ_Cloud> grid;
	grid.setLeafSize(0.03f, 0.03f, 0.03f);
	grid.setInputCloud(cloud);
	grid.filter(*cloud);
	return cloud;
}

int PCL_ICP(cloud_pointer& cloud1, cloud_pointer& cloud2)
{
	float dist;

	// PCL ICP for computation of the transformation matrix
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cout << "\nSTARTING ICP..." << endl;

	// Set leaf size
	voxelleaf(cloud1);
	voxelleaf(cloud2);

	// Setting a initial transformation estimate
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	// Define a translation of specified distance centimeters on the Z axis.
	cout << "\nEnter translation distance in centimeters on the Z axis\t";
	cin >> dist;
	transform_2.translation() << 0.0, 0.0, dist/100;
	// The same rotation matrix as before; theta radians around Z axis
	//transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud(*cloud1, *cloud1, transform_2);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations(50);
	icp.setInputSource(cloud1);
	icp.setInputTarget(cloud2);
	icp.setTransformationRotationEpsilon(0.9999);
	icp.align(*cloud);
	if (icp.hasConverged())
	{
		cout << "\nFinal transformation matrix is : \n\n" << icp.getFinalTransformation() << endl;
		float x, y, z, roll, pitch, yaw;
		auto trafo = icp.getFinalTransformation();
		Eigen::Transform<float, 3, Eigen::Affine> tROTA(trafo);
		pcl::getTranslationAndEulerAngles(tROTA, x, y, z, roll, pitch, yaw);
		cout << "\nTranslational Elements:\n\n\tx\t" << x * 100 << "\n\ty\t" << y * 100 << "\n\tz\t" << (z * 100) + dist << endl;
		cout << "\nRotational Elements:\n\n\tRoll\t" << roll * 57.29577 << "\n\tPitch\t" << pitch * 57.29577 << "\n\tYaw\t" << yaw * 57.29577 << endl;
		cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
		if (abs(roll) > 0.174533 || abs(pitch) > 0.174533)
			cout << "\nObject has deviated more than 10 degrees!!" << endl;
		return 0;
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		std::cout << "\nPress Enter to exit Program... ";
		return (-1);
	}

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

	//====================
	// Object Declaration
	//====================
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	// Declare object to store Point Cloud
	std::vector<cloud_pointer> p_cloud;

	// Obtain a list of devices currently present on the system
	
	// Create context
	rs2::context ctx;
	
	// Get device
	auto devices = ctx.query_devices();
	size_t device_count = devices.size();
	if (!device_count)
	{
		cout << "No device detected. Is it plugged in?\n";
		return EXIT_SUCCESS;
	}

	// Get the first connected device
	auto dev = devices[0];

	// Enter advanced mode
	if (dev.is<rs400::advanced_mode>())
	{
		// Get the advanced mode functionality
		auto advanced_mode_dev = dev.as<rs400::advanced_mode>();

		// Load and configure .json file to device
		ifstream t("Default 424x240.json");
		string str((istreambuf_iterator<char>(t)), istreambuf_iterator<char>());
		advanced_mode_dev.load_json(str);
	}
	else
	{
		cout << "Current device doesn't support advanced-mode!\n";
		return EXIT_FAILURE;
	}

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;

	//Contruct a pipeline which abstracts the device

	rs2::device selected_device = dev;
	auto depth_sensor = selected_device.first<rs2::depth_sensor>();

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;


	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;


	//======================
	// Stream configuration
	//======================
	cfg.enable_stream(RS2_STREAM_INFRARED, 424, 240, RS2_FORMAT_Y8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 30);

	rs2::pipeline_profile selection = pipe.start(cfg);


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

		// Generate Point Cloud
		auto points = pc.calculate(depth);

		// Convert generated Point Cloud to PCL Formatting
		cloud_pointer cloud = PCL_Conversion(points);

		//========================================
		// Filter PointCloud (PassThrough Method)
		//========================================
		if (i == 1) {
			pcl::PassThrough<pcl::PointXYZ> Cloud_Filter; // Create the filtering object
			Cloud_Filter.setInputCloud(cloud);           // Input generated cloud to filter
			Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
			Cloud_Filter.setFilterLimits(0.0, 0.45);      // Set accepted interval values
			Cloud_Filter.filter(*cloud);              // Filtered Cloud Outputted
		}
		p_cloud.push_back(cloud);
		Load_PCDFile(cloud);
		cout << "Pointcloud successfully generated. " << endl;

		i++; // Increment File Name

	}//End-while

	PCL_ICP(p_cloud[0], p_cloud[1]); // Run ICP on the pointclouds

	std::system("pause");

	return EXIT_SUCCESS;
}