# Master_Project
angular tilt of objects

Background removal
http://www.pcl-users.org/Background-removal-td4035780.html

Plane model segmentation
http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation

Euclidean Cluster Extraction
http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

3D Object Recognition based on Correspondence Grouping
http://www.pointclouds.org/documentation/tutorials/correspondence_grouping.php

Sample .pcd files
- https://github.com/PointCloudLibrary/data/tree/master/segmentation/mOSD/test
- https://github.com/jvgomez/irml/tree/master/pointclouds/database/3dmodelsviews

Segmentation
	Color-based region growing segmentation
	http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php#region-growing-rgb-segmentation

	Region growing segmentation
	http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php#region-growing-segmentation

	Identifying ground returns using ProgressiveMorphologicalFilter segmentation
	http://pointclouds.org/documentation/tutorials/progressive_morphological_filtering.php#progressive-morphological-filtering

	Conditional Euclidean Clustering
	http://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.php#conditional-euclidean-clustering

	Min-Cut Based Segmentation
	http://pointclouds.org/documentation/tutorials/min_cut_segmentation.php#min-cut-segmentation



Header files (ICP, correspondence grouping):
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

// Correspondence grouping Headers
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
