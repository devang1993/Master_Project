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
	grid.setLeafSize(0.03f, 0.03f, 0.03f);
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
		if (roll > 0.174533 || pitch > 0.174533)
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

double
computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

//============================
//Euclidean Cluster Extraxtion
//============================

/*cloud_pointer cluster(cloud_pointer& cloud)
{

	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

	pcl::VoxelGrid<RGB_Cloud> grid;
	grid.setLeafSize(0.005f, 0.005f, 0.005f);
	grid.setInputCloud(cloud);
	grid.filter(*cloud);

	std::cout << "PointCloud after filtering has: " << cloud->points.size() << " data points." << std::endl; //*

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGBA>());
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
		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGBA>);
		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud = *cloud_f;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(0.01); // 2cm
	ec.setMinClusterSize(10000);
	ec.setMaxClusterSize(60000); //init 25,000
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		/*
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_cluster, false); //*

		j++;
	}
	Load_PCDFile(cloud);
	return cloud;
}*/

//==========================
// 3D Corresponence Grouping
//==========================

//int grouping3d(cloud_pointer& cloud1, cloud_pointer& cloud2) {
//
//	 Euclidean Cluster Extraction
//	cluster(cloud1);
//	cluster(cloud2);
//
//	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
//	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
//	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
//	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
//	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
//	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
//	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
//	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());
//
//	model = cloud1;
//	scene = cloud2;
//
//	==============================
//	  Set up resolution invariance
//	==============================
//	if (use_cloud_resolution_)
//	{
//		float resolution = static_cast<float> (computeCloudResolution(model));
//		if (resolution != 0.0f)
//		{
//			model_ss_ *= resolution;
//			scene_ss_ *= resolution;
//			rf_rad_ *= resolution;
//			descr_rad_ *= resolution;
//			cg_size_ *= resolution;
//		}
//
//		std::cout << "Model resolution:       " << resolution << std::endl;
//		std::cout << "Model sampling size:    " << model_ss_ << std::endl;
//		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
//		std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
//		std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
//		std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
//	}
//
//	=================
//	  Compute Normals
//	=================
//
//	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
//	norm_est.setKSearch(10);
//	norm_est.setInputCloud(model);
//	norm_est.compute(*model_normals);
//
//	norm_est.setInputCloud(scene);
//	norm_est.compute(*scene_normals);
//
//	========================================
//	  Downsample Clouds to Extract keypoints
//	========================================
//
//	pcl::UniformSampling<PointType> uniform_sampling;
//	uniform_sampling.setInputCloud(model);
//	uniform_sampling.setRadiusSearch(model_ss_);
//	uniform_sampling.filter(*model_keypoints);
//	std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;
//
//	uniform_sampling.setInputCloud(scene);
//	uniform_sampling.setRadiusSearch(scene_ss_);
//	uniform_sampling.filter(*scene_keypoints);
//	std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;
//
//	==================================
//	  Compute Descriptor for keypoints
//	==================================
//
//	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
//	descr_est.setRadiusSearch(descr_rad_);
//
//	descr_est.setInputCloud(model_keypoints);
//	descr_est.setInputNormals(model_normals);
//	descr_est.setSearchSurface(model);
//	descr_est.compute(*model_descriptors);
//
//	descr_est.setInputCloud(scene_keypoints);
//	descr_est.setInputNormals(scene_normals);
//	descr_est.setSearchSurface(scene);
//	descr_est.compute(*scene_descriptors);
//
//	==============================================
//	  Find Model-Scene Correspondences with KdTree
//	==============================================
//
//	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
//
//	pcl::KdTreeFLANN<DescriptorType> match_search;
//	match_search.setInputCloud(model_descriptors);
//
//	  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
//	for (size_t i = 0; i < scene_descriptors->size(); ++i)
//	{
//		std::vector<int> neigh_indices(1);
//		std::vector<float> neigh_sqr_dists(1);
//		if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
//		{
//			continue;
//		}
//		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
//		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
//		{
//			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
//			model_scene_corrs->push_back(corr);
//		}
//	}
//	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
//
//	===================
//	  Actual Clustering
//	===================
//	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
//	std::vector<pcl::Correspondences> clustered_corrs;
//
//	===============
//	  Using Hough3D
//	===============
//
//	if (use_hough_)
//	{
//		=====================================================
//		  Compute (Keypoints) Reference Frames only for Hough
//		=====================================================
//
//		pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
//		pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());
//
//		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
//		rf_est.setFindHoles(true);
//		rf_est.setRadiusSearch(rf_rad_);
//
//		rf_est.setInputCloud(model_keypoints);
//		rf_est.setInputNormals(model_normals);
//		rf_est.setSearchSurface(model);
//		rf_est.compute(*model_rf);
//
//		rf_est.setInputCloud(scene_keypoints);
//		rf_est.setInputNormals(scene_normals);
//		rf_est.setSearchSurface(scene);
//		rf_est.compute(*scene_rf);
//
//		  Clustering
//		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
//		clusterer.setHoughBinSize(cg_size_);
//		clusterer.setHoughThreshold(cg_thresh_);
//		clusterer.setUseInterpolation(true);
//		clusterer.setUseDistanceWeight(false);
//
//		clusterer.setInputCloud(model_keypoints);
//		clusterer.setInputRf(model_rf);
//		clusterer.setSceneCloud(scene_keypoints);
//		clusterer.setSceneRf(scene_rf);
//		clusterer.setModelSceneCorrespondences(model_scene_corrs);
//
//		clusterer.cluster (clustered_corrs);
//		clusterer.recognize(rototranslations, clustered_corrs);
//	}
//	else // Using GeometricConsistency
//	{
//		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
//		gc_clusterer.setGCSize(cg_size_);
//		gc_clusterer.setGCThreshold(cg_thresh_);
//
//		gc_clusterer.setInputCloud(model_keypoints);
//		gc_clusterer.setSceneCloud(scene_keypoints);
//		gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);
//
//		gc_clusterer.cluster (clustered_corrs);
//		gc_clusterer.recognize(rototranslations, clustered_corrs);
//	}
//
//	=================
//	  Output results
//	=================
//
//	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
//	for (size_t i = 0; i < rototranslations.size(); ++i)
//	{
//		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
//		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;
//
//		 Print the rotation matrix and translation vector
//		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
//		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);
//
//		printf("\n");
//		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
//		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
//		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
//		printf("\n");
//		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
//	}
//
//	===============
//	  Visualization
//	===============
//
//	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
//	viewer.addPointCloud(scene, "scene_cloud");
//
//	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
//	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());
//
//	if (show_correspondences_ || show_keypoints_)
//	{
//		  We are translating the model so that it doesn't end in the middle of the scene representation
//		pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
//		pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
//
//		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
//		viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
//	}
//
//	if (show_keypoints_)
//	{
//		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
//		viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
//		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
//
//		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
//		viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
//		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
//	}
//
//	for (size_t i = 0; i < rototranslations.size(); ++i)
//	{
//		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
//		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);
//
//		std::stringstream ss_cloud;
//		ss_cloud << "instance" << i;
//
//		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
//		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());
//
//		if (show_correspondences_)
//		{
//			for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
//			{
//				std::stringstream ss_line;
//				ss_line << "correspondence_line" << i << "_" << j;
//				PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
//				PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);
//
//				  We are drawing a line for each pair of clustered correspondences found between the model and the scene
//				viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
//			}
//		}
//	}
//
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//	}
//	system("pause");
//	return (0);
//
//}

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

			//========================================
			// Filter PointCloud (PassThrough Method)
			pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
			Cloud_Filter.setInputCloud(cloud);           // Input generated cloud to filter
			Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
			Cloud_Filter.setFilterLimits(0.0, 0.49);      // Set accepted interval values
			Cloud_Filter.filter(*cloud);              // Filtered Cloud Outputted
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