// 	Intel Realsense Headers
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

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
    while (true)
    {

        std::cout << "Press any key to capture" << std::endl;
        std::cin.get();

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
            if (Vertex[i].z <= 0.72)
            {
                // Mapping Depth data to XYZ values
                cloud->points[i].x = Vertex[i].x;
                cloud->points[i].y = Vertex[i].y;
                cloud->points[i].z = Vertex[i].z;
            }
        }
        
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
        std::cout << std::endl;
        std::cout << "Press [Q] in viewer to continue. " << std::endl;

        // Allow user to rotate point cloud and view it
        viewer->spin();

        // Save PCD File
        pcl::io::savePCDFile("Front_View.pcd", *cloud);
    }

    return 0;
}