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

#include <pcl/registration/ia_ransac.h>

// 3D Correspondence Grouping
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>

using namespace std;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;


// Prototypes
void Load_PCDFile(cloud_pointer& cloud);
bool userInput(void);
cloud_pointer voxelleaf(cloud_pointer& cloud);
int PCL_ICP(cloud_pointer& cloud1, cloud_pointer& cloud2);
//cloud_pointer cluster(cloud_pointer& cloud);
//int grouping3d(cloud_pointer& cloud1, cloud_pointer& cloud2);

//Algorithm params
bool show_keypoints_(false);
bool show_correspondences_(false);
bool use_cloud_resolution_(false);
bool use_hough_(true);
float model_ss_(0.01f);
float scene_ss_(0.03f);
float rf_rad_(0.015f);
float descr_rad_(0.02f);
float cg_size_(0.01f);
float cg_thresh_(5.0f);

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

cloud_pointer voxelleaf(cloud_pointer& cloud) {
	pcl::VoxelGrid<RGB_Cloud> grid;
	grid.setLeafSize(0.01f, 0.01f, 0.01f);
	grid.setInputCloud(cloud);
	grid.filter(*cloud);
	Load_PCDFile(cloud);
	return cloud;
}

int PCL_ICP(cloud_pointer& cloud1, cloud_pointer& cloud2)
{
	// PCL ICP for computation of the transformation matrix

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cout << "\nstarting icp" << endl;

	// Euclidean Cluster Extraction
	//cluster(cloud1);
	//cluster(cloud2);

	voxelleaf(cloud1);
	voxelleaf(cloud2);


	//// Setting a initial transformation estimate
	//Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	//// Define a translation of 6 centimeters on the x axis.
	//transform_2.translation() << 0.0, 0.0, 0.06;
	//// The same rotation matrix as before; theta radians around Z axis
	////transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	//// Print the transformation
	//printf("\nMethod #2: using an Affine3f\n");
	//std::cout << transform_2.matrix() << std::endl;
	//// Executing the transformation
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	//// You can either apply transform_1 or transform_2; they are the same
	//pcl::transformPointCloud(*cloud1, *cloud1, transform_2);
	//Load_PCDFile(cloud1);

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	cout << "\nset Maximum number of iterations:   ";
	cin >> i;
	icp.setMaximumIterations(i);
	icp.setInputSource(cloud1);
	icp.setInputTarget(cloud2);
	icp.setTransformationRotationEpsilon(0.9999);
	icp.align(*cloud);
	Load_PCDFile(cloud);
	if (icp.hasConverged())
	{
		cout << "\n Final transformation matrix is : \n" << icp.getFinalTransformation() << endl;
		float x, y, z, roll, pitch, yaw;
		auto trafo = icp.getFinalTransformation();
		Eigen::Transform<float, 3, Eigen::Affine> tROTA(trafo);
		pcl::getTranslationAndEulerAngles(tROTA, x, y, z, roll, pitch, yaw);
		cout << "Translational Elements:       x           y           z " << endl;
		cout << "                          " << x << "  " << y << "  " << z << endl;
		cout << "Rotational Elements:       Roll      Pitch          Yaw " << endl;
		cout << "                          " << roll * 57.29577 << "    " << pitch * 57.29577 << "   " << yaw * 57.29577;
		cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
		if (abs(roll) > 0.174533 || abs(pitch) > 0.174533)
			cout << "Object has deviated more than 10 degrees!!" << endl;
		return EXIT_SUCCESS;
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
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	std::vector<cloud_pointer> p_cloud;

	char keyin;
	std::cout << "Enter Y to capture, N to load saved PCDs; " << endl;
	cin >> keyin;
	if (keyin == 'Y' || keyin == 'y') {

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

			Load_PCDFile(cloud);

			//========================================
			// Filter PointCloud (PassThrough Method)
			if (i == 1) {
				pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
				Cloud_Filter.setInputCloud(cloud);           // Input generated cloud to filter
				Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
				Cloud_Filter.setFilterLimits(0.0, 0.46);      // Set accepted interval values
				Cloud_Filter.filter(*cloud);              // Filtered Cloud Outputted
			}
			p_cloud.push_back(cloud);

			cout << " Pointcloud successfully generated. " << endl;

			//Load generated PCD file for viewing
			Load_PCDFile(p_cloud.at(i - 1));
			i++; // Increment File Name

		}//End-while
	}

	else if (keyin == 'N' || keyin == 'n') {
		float h;
		while (i < 3) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			cout << "\nReading Pointcloud" << i << endl;
			pcl::io::loadPCDFile("Captured_Frame" + to_string(i) + ".pcd", *cloud);
			Load_PCDFile(cloud);

			//========================================
			// Filter PointCloud (PassThrough Method)
			//========================================
			pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
			Cloud_Filter.setInputCloud(cloud);           // Input generated cloud to filter
			Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
			if (i == 1)	h = 0.41f;
			else h = 0.485f;
			Cloud_Filter.setFilterLimits(0.0, h);      // Set accepted interval values
			Cloud_Filter.filter(*cloud);              // Filtered Cloud Outputted

			p_cloud.push_back(cloud);
			Load_PCDFile(p_cloud.at(i - 1));
			i++;
		}
	}
	else {
		return (-1);
	}

	PCL_ICP(p_cloud[0], p_cloud[1]); // Run ICP on the pointclouds

	std::system("pause");

	return EXIT_SUCCESS;
}