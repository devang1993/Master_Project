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

using namespace std;

typedef pcl::PointXYZ XYZ_Cloud;
typedef pcl::PointCloud<XYZ_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;

// Prototypes
cloud_pointer PCL_Conversion(const rs2::points& points);
bool userInput(void);
void PCL_ICP(void);

// Global Variables
int i = 1; // Index for incremental file name
std::vector<cloud_pointer> p_cloud;

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

	return cloud; // PCL RGB Point Cloud generated
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

//========================================
// ICP to compute the rigid transformation
//========================================

void PCL_ICP(void)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cout << "\nstarting icp" << endl;

	//pcl::VoxelGrid<XYZ_Cloud> grid;
	//grid.setLeafSize(0.5, 0.5, 0.5);
	//grid.setInputCloud(p_cloud.at(0));
	//grid.filter(*p_cloud.at(0));

	//grid.setInputCloud(p_cloud.at(1));
	//grid.filter(*p_cloud.at(1));

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	cout << "\nset Maximum number of iterations:\t";
	cin >> i;
	icp.setMaximumIterations(i);
	icp.setInputSource(p_cloud.at(1));
	icp.setInputTarget(p_cloud.at(0));
	icp.setTransformationRotationEpsilon(0.95);
	icp.setMaxCorrespondenceDistance(0.5);
	icp.align(*cloud);

	// Specify convergence statements
	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\n Final transformation matrix is : \n" << icp.getFinalTransformation();
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		std::cout << "\nPress Enter to exit Program... ";
	}

}

int main()
{
	//======================
	// Variable Declaration
	//======================
	bool captureLoop = true; // Loop control for generating point clouds

	//====================
	// Object Declaration
	//====================
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
		pcl::PassThrough<pcl::PointXYZ> Cloud_Filter; // Create the filtering object
		Cloud_Filter.setInputCloud(cloud);           // Input generated cloud to filter
		Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
		Cloud_Filter.setFilterLimits(0.0, 2.0);      // Set accepted interval values
		Cloud_Filter.filter(*cloud);              // Filtered Cloud Outputted

		p_cloud.push_back(cloud);

		cout << "Pointcloud successfully generated. " << endl;

		i++; // Increment File Name

	}//End-while

	PCL_ICP();	// Function call to perform Rigid transformation

	system("pause");

	return EXIT_SUCCESS;
}
