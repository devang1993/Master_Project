/*
====================================================================================================
====================================================================================================
			DETECTION OF ROTATIONAL DEVIATION OF OBJECTS USING A 3D DEPTH CAMERA
====================================================================================================
====================================================================================================
v1.0
Authors: Devang Mehta, Siddharth Baburaj

BRIEF:
Detection of rotational deviation between two objects represented by point clouds
by means of implementing the Iterative Closest Point (ICP) Algorithm. The 3D imaging
device used here is the Intel RealSense Depth Camera D415.

THEORY:
Point clouds are an efficient way of representing the depth data, and optional colour
data of images captured by a 3D camera/scanner. Alternatively a point cloud can be defined
as a collection of data points within an image defined by a given coordinate system.
The Point Cloud Library (PCL), a open-source library for 2D/3D image processing,
is extensively used within this code, in order to facilitate several algorithm key
points, such as point cloud I/O, post-processing, point cloud registration, etc.

The key algorithm used herein is called Iterative Closest Point (ICP).
ICP is more commonly implemented in scenarios where point cloud images captured of
different orientations of the same environment are needed to be 'stitched' or merged
into a single seamless 3D image. This requires every set of point cloud datasets
acquired from different views to be aligned into a single point cloud model,
corresponding to a global coordinate framework. Hence, ICP aims to find the
transformation between a point cloud and some reference point cloud, by minimizing
the square errors between the corresponding entities => find optimal the rotation and
translation matrices to transform source point cloud P2 to target point cloud P1.

The ICP provides output as a 4x4 homogenous transformation matrix between the original
'reference' point cloud and the test 'deviated' point cloud. The angular deviation in
Euler angles as well as the overall translational difference can be retrieved
from this matrix.
==================================================================================================*/



/*==================================================================================================
		HEADER SECTION, PREPROCESSOR DIRECTIVES
==================================================================================================*/

//	CPP Headers, Standard Libraries
#include <iostream>
#include <algorithm> 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>
#include <stdlib.h>

// 	Intel Realsense Headers
#include <librealsense2/rs.hpp> 
#include <librealsense2/rs_advanced_mode.hpp>

// 	Point Cloud Library Headers
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

/*==================================================================================================
		GLOBAL VARIABLES, TYPE DEFINITIONS, FUNCTION PROTOTYPES
==================================================================================================*/

// Global Variables
int i = 1; // Index for incremental file name

//Type Definitions
typedef pcl::PointXYZ XYZ_Cloud;
typedef pcl::PointCloud<XYZ_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
using namespace std;

// Prototypes
bool userInput(void);
cloud_pointer voxelleaf(cloud_pointer& cloud);
int PCL_ICP(cloud_pointer& cloud1, cloud_pointer& cloud2);
void Load_PCDFile(cloud_pointer& cloud);

/*==================================================================================================
		MAIN CODE
==================================================================================================*/


/*===============================================
Visualization of point cloud passed onto function
using the PCL Visualization library
================================================*/
void Load_PCDFile(cloud_pointer& cloud)
{
	// Create viewer object titled "Captured Frame"
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Captured Frame"));

	// Set background of viewer to black
	viewer->setBackgroundColor(0, 0, 0);

	// Add generated point cloud and identify with string "Cloud"
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "Cloud");

	// Setting default size for rendered points
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");

	// Initialize camera parameters with some default values.
	viewer->initCameraParameters();

	// Note: No method to close PC visualizer, pressing Q to continue software flow only solution.
	cout << endl;
	cout << "Press [Q] in viewer to continue. " << endl;

	// Allow user to rotate point cloud and view it
	viewer->spin();
}


/*===================================================
Function is utilized to fill a point cloud
object with depth and RGB data from a single
frame captured using the Realsense.
===================================================*/
cloud_pointer PCL_Conversion(const rs2::points& points)
{
	// Object Declaration (Point Cloud)
	cloud_pointer cloud(new point_cloud);

	// Convert data captured from Realsense camera to Point Cloud
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto Texture_Coord = points.get_texture_coordinates();
	auto Vertex = points.get_vertices();

	// Iterating through all points and setting XYZ coordinates
	for (int i = 0; i < points.size(); i++)
	{
		// Mapping Depth data to XYZ values
		cloud->points[i].x = Vertex[i].x;
		cloud->points[i].y = Vertex[i].y;
		cloud->points[i].z = Vertex[i].z;
	}
	// returns the generated PCL XYZ Point Cloud
	return cloud;
}


/*========================================
Prompts user for a character input to
facilitate decision making.
[y|Y] - Capture frame and save as .pcd
[n|N] - Exit program
========================================*/
bool userInput(void)
{
	// Declare and initialize flags to access user input(s)
	bool setLoopFlag;
	bool inputCheck = false;
	char takeFrame;
	do {
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


/*========================================
Post-processing implemented on passed
point cloud, by means of increasing the voxel
size of each point, thereby decreasing the
resoultion. This increases efficiency in
execution of the ICP algorithm later on.
========================================*/
cloud_pointer voxelleaf(cloud_pointer& cloud)
{
	pcl::VoxelGrid<XYZ_Cloud> grid;
	// Setting desired voxel size
	grid.setLeafSize(0.03f, 0.03f, 0.03f);
	grid.setInputCloud(cloud);
	grid.filter(*cloud);
	// return 'filtered' point cloud
	return cloud;
}


/*========================================
Implementation of the Iterative Closest Point algorithm
on the passed point clouds (cloud 1 - target, cloud 2 - source)
Produces an optimal homogenous transformation  matrix between
the two, in addition to displaying the translational elements
as well as the Euler angles. The iterative process is further
sped up by providing an initial 'guess' transformation matrix,
wherein the user is asked to key in the approximate translational
distance in cm, through which the object may have moved in the
source image with respect to the target.
========================================*/
int PCL_ICP(cloud_pointer& cloud1, cloud_pointer& cloud2)
{
	// Create a point cloud pointer to store the final aligned point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cout << "\nSTARTING ICP..." << endl;

	// Set leaf size, by passing clouds to post-processing function voxelleaf
	voxelleaf(cloud1);
	Load_PCDFile(cloud1);
	voxelleaf(cloud2);
	Load_PCDFile(cloud2);

	// Setting a initial transformation estimate
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	// Define a translation of specified distance centimeters on the Z axis.
	float dist;
	cout << "\nEnter translation distance in centimeters on the Z axis\t";
	cin >> dist;
	transform.translation() << 0.0, 0.0, dist / 100;

	// Transforming the source point cloud using above transformation matrix
	pcl::transformPointCloud(*cloud1, *cloud1, transform);

	// Create object for ICP, templated with Point Cloud XYZ data (see type def)
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	// Set max number of iterations required for ICP, before convergence
	icp.setMaximumIterations(50);

	//maximum allowable translation squared difference between two consecutive transformations
	icp.setTransformationEpsilon(0.005);

	//maximum allowable rotation difference between two consecutive transformations
	icp.setTransformationRotationEpsilon(0.999);

	// Set source and target point clouds
	icp.setInputSource(cloud1);
	icp.setInputTarget(cloud2);

	// Set maximum allowable difference between two consecutive transformations
	// (epsilon is the cos(angle) in a axis-angle representation)
	//icp.setTransformationRotationEpsilon(0.9999);

	// Iterative process enacted here
	icp.align(*cloud);

	// Check convergence of process
	if (icp.hasConverged())
	{
		// Displays final homogenous transformation matrix upon satisfying convergence condition
		cout << "\nFinal transformation matrix is : \n\n" << icp.getFinalTransformation() << endl;

		// Variables declared to store translational elements and Euler angles
		float x, y, z, roll, pitch, yaw;

		// Isolation of translational and rotational elements 
		auto trafo = icp.getFinalTransformation();
		Eigen::Transform<float, 3, Eigen::Affine> tROTA(trafo);
		pcl::getTranslationAndEulerAngles(tROTA, x, y, z, roll, pitch, yaw);
		cout << "\nTranslational Elements:\n\n\tx\t" << x * 100 << "\n\ty\t" << y * 100 << "\n\tz\t" << (z * 100) + dist << endl;
		cout << "\nRotational Elements:\n\n\tRoll\t" << roll * 57.29577 << "\n\tPitch\t" << pitch * 57.29577 << "\n\tYaw\t" << yaw * 57.29577 << endl;

		// Display convergence or fitness score of ICP algorithm
		cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;

		// Display warning message if deviation crosses nominal range
		if (abs(roll) > 0.174533 || abs(pitch) > 0.174533)
			cout << "\nObject has deviated more than 10 degrees!!" << endl;
		return 0;
	}
	else
	{
		// Display error message if ICP could not converge, threshold parameters have to be changed
		PCL_ERROR("\nICP has not converged.\n");
		std::cout << "\nPress Enter to exit Program... ";
		return (-1);
	}

}


/*========================================
------------ Main Function ---------------
========================================*/
int main()
{
	char userkey = 'Y';
	do {
		// Declare variable for loopm control (point cloud generation)
		bool captureLoop = true;

		// Declare PCL object for point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		// Declare a vector to store consecutive point clouds
		std::vector<cloud_pointer> p_cloud;

		// Obtain imaging device details, check if device is connected
		rs2::context ctx;
		auto devices = ctx.query_devices();
		size_t device_count = devices.size();
		if (!device_count)
		{
			cout << "No device detected. Is it plugged in?\n";
			while (!device_count) {
				auto devices = ctx.query_devices();
				size_t device_count = devices.size();
			}
		}

		// Get the first connected device
		auto dev = devices[0];
		// Enter advanced mode, obtain saved camera configuration file (JSON)
		if (dev.is<rs400::advanced_mode>())
		{
			// Get the advanced mode functionality
			auto advanced_mode_dev = dev.as<rs400::advanced_mode>();

			// Load and configure .json file to device
			// Loaded config file - maximum depth resolution (depth units: 0,0001m) and hole-filling
			ifstream t("ROI captured1_depth4.json");
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

		// Create a configuration object for configuring the pipeline with a non default profile
		rs2::config cfg;

		// Stream configuration
		cfg.enable_stream(RS2_STREAM_INFRARED, 424, 240, RS2_FORMAT_Y8, 30);
		cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 30);

		// Begin streaming with updated configuration
		rs2::pipeline_profile selection = pipe.start(cfg);


		// Capture point clouds based on user input, for 2 consecutive point clouds
		while (captureLoop == true && i < 3) {

			// Set loop flag based on user input
			captureLoop = userInput();
			if (captureLoop == false) { break; }

			// Wait for frames from the camera to settle
			for (int i = 0; i < 30; i++)
			{
				// Drop several frames for auto-exposure
				auto frames = pipe.wait_for_frames();
			}

			// Capture a single frame and obtain depth + RGB values from it    
			auto frames = pipe.wait_for_frames();
			auto depth = frames.get_depth_frame();

			// Generate points from depth data
			auto points = pc.calculate(depth);

			// Convert generated Point Cloud to PCL Formatting
			cloud_pointer cloud = PCL_Conversion(points);

			// Background removal using a 'PassThrough' point cloud filter
			float z;
			if (i == 1)
				z = 0.36;
			else
				z = 0.51;
			// Create the filtering object
			pcl::PassThrough<pcl::PointXYZ> Cloud_Filter;

			// Input generated cloud to filter
			Cloud_Filter.setInputCloud(cloud);

			// Set field name to Z-coordinate
			Cloud_Filter.setFilterFieldName("z");

			// Set accepted interval values (m), beyond which points nullified
			Cloud_Filter.setFilterLimits(0.0, z);

			// Filtered Cloud Outputted
			Cloud_Filter.filter(*cloud);

			// Store point cloud into vector
			p_cloud.push_back(cloud);

			// Visualization of generated point cloud
			Load_PCDFile(cloud);
			cout << "Pointcloud successfully generated. " << endl;

			// Increment File Name
			i++;
		}

		// Run ICP on the generated point clouds
		PCL_ICP(p_cloud[0], p_cloud[1]);

		std::system("pause");
		cout << "Would you like to continue? (Y/N)" << endl;
		cin >> userkey;
		if (userkey == 'N' || userkey == 'n') { break; }
		std::vector<cloud_pointer>().swap(p_cloud);
	} while (userkey == 'Y' || userkey == 'y');
	return EXIT_SUCCESS;
}
