#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <thread>
#include <Eigen/Dense> // For Eigen::Vector4f

// Color structure to hold RGB values
struct Color{
	float r, g, b;

	Color(float setR, float setG, float setB)
		: r(setR), g(setG), b(setB)
	{}
};

// Function to read a PCD file
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr readPCD(std::string path){

    typename pcl::PointCloud<PointT>::Ptr cloud (new typename pcl::PointCloud<PointT>());

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Read the point cloud data
    if (pcl::io::loadPCDFile<PointT>(path, *cloud) == -1) {
        PCL_ERROR("Couldn't read pcd file... \n");
        return nullptr;
    }

    return cloud;
}

// Function to filter a point cloud
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filterPCD(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){

    typename pcl::PointCloud<PointT>::Ptr cloud_box (new typename pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new typename pcl::PointCloud<PointT>());

    // Create and configure the crop box filter
    typename pcl::CropBox<PointT> cropBoxFilter(true);
    cropBoxFilter.setInputCloud(cloud);
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    
    // Apply the crop box filter
    cropBoxFilter.filter(*cloud_box);

    // Downsampling a PointCloud using a VoxelGrid filter
    // Create the filtering object
    typename pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud_box);
    vox.setLeafSize(filterRes,filterRes,filterRes);
    vox.filter(*cloud_filtered);

    return cloud_filtered;
}

// Function to visualize a point cloud
template <typename PointT>
void visualizePCD(pcl::visualization::PCLVisualizer::Ptr& viewer, typename pcl::PointCloud<PointT>::Ptr cloud, const std::string& name, Color color){
    if (color.r == -1)
    {
        // Create a color handler that colors points based on their intensity
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
        // Add the point cloud to the viewer with the intensity color handler
        viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
    }
    else
    {
        // Add the point cloud to the viewer with a default color handler
        viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
        // Set the color of the point cloud to the specified RGB values
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    }
    
    // Set the point size property for the point cloud
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

// Function to segment a plane from a point cloud
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> planeSegmentation(typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations, float threshold){
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    typename pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold (threshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size() == 0){
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    }

    // point clouds to hold segmented regions
    typename pcl::PointCloud<PointT>::Ptr cloud_obs (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>());

    for(auto index: inliers->indices){
        cloud_plane->points.push_back(cloud->points[index]);
    }

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_obs);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmented_pcls(cloud_obs, cloud_plane);

    return segmented_pcls; 
}

// Function to perform Euclidean cluster segmentation
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanClusterSegmentation(typename pcl::PointCloud<PointT>::Ptr cloud_obs, float cluster_tolerance, int min_size, int max_size) {
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_obs);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_obs);
    ec.extract(cluster_indices);

    // Extract each cluster and add to clusters vector
    for (const auto& indices : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto& idx : indices.indices)
            cloud_cluster->points.push_back(cloud_obs->points[idx]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    return clusters;
}


// Function to get bounding box dimensions
template <typename PointT>
std::pair<PointT , PointT> bounding_box(typename pcl::PointCloud<PointT>::Ptr cluster){
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    std::pair<PointT , PointT> points(minPoint, maxPoint);

    return points;
}

// Function to render a bounding box
template <typename PointT>
void render_box(pcl::visualization::PCLVisualizer::Ptr& viewer, std::pair<PointT , PointT> points, int id, Color color, float opacity){
    if (opacity > 1.0)
        opacity = 1.0;
    if (opacity < 0.0)
        opacity = 0.0;

    // Create unique identifiers for wireframe and filled box representations
    std::string wireframeCube = "box" + std::to_string(id);
    std::string filledCube = "boxFill" + std::to_string(id);

    PointT minPoint = points.first;
    PointT maxPoint = points.second;

    // Add wireframe box
    viewer->addCube(minPoint.x, maxPoint.x, minPoint.y, maxPoint.y, minPoint.z, maxPoint.z,
                    color.r, color.g, color.b, wireframeCube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                        wireframeCube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, wireframeCube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, wireframeCube);

    // Add filled transparent box
    viewer->addCube(minPoint.x, maxPoint.x, minPoint.y, maxPoint.y, minPoint.z, maxPoint.z,
                    color.r, color.g, color.b, filledCube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                        filledCube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, filledCube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, filledCube);
}

// Function to stream PCD files from a directory
std::vector<boost::filesystem::path> streamPcd(std::string dataPath)
{
    // Create a vector to store paths to PCD files
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // Sort files in ascending order for chronological playback
    sort(paths.begin(), paths.end());

    return paths;
}

// Function to process a PCD file and visualize it
void process_pcd(pcl::visualization::PCLVisualizer::Ptr& viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());

    // apply filtering 
    cloud_filtered = filterPCD<pcl::PointXYZI>(cloud, 0.2, Eigen::Vector4f(-15, -6.0, -3, 1), Eigen::Vector4f(30, 6.0, 10, 1));

    // perform plane segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_pcls = planeSegmentation<pcl::PointXYZI>(cloud_filtered, 1000, 0.2);

    // perform Euclidean cluster segmentation
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = euclideanClusterSegmentation<pcl::PointXYZI>(segmented_pcls.first, 0.5, 10, 600);

    // visualize the plane
    if (!segmented_pcls.second->empty()) {
        visualizePCD<pcl::PointXYZI>(viewer, segmented_pcls.second, "Cloud_Plane", Color(0,1,0));
    } else {
        std::cout << "No plane detected!" << std::endl;
    }

    // visualizePCD<pcl::PointXYZI>(viewer, cloud_filtered, "Filtered_Cloud", Color(0,0,1));

    // Visualize clusters and bounding boxes
    int cluster_id=0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1), Color(0, 1, 1)};

    for(auto cluster: clusters){
        visualizePCD<pcl::PointXYZI>(viewer, cluster, "Cloud_Obstacle"+std::to_string(cluster_id), colors[cluster_id % (colors.size())]);
        auto points = bounding_box<pcl::PointXYZI>(cluster);
        render_box<pcl::PointXYZI>(viewer, points, cluster_id, Color(0.5, 0.5, 0.5), 0.5);
        ++cluster_id;
    }
}

int main() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    // Create the visualizer object
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    std::vector<boost::filesystem::path> stream = streamPcd("../data/");
    auto streamIterator = stream.begin();

    // Define desired FPS and delay in milliseconds
    int fps = 30;
    int delay = 1000 / fps; 

    while (!viewer->wasStopped())
    {
        // Clear previous point clouds and shapes from viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load PCD file and process
        cloud = readPCD<pcl::PointXYZI>((*streamIterator).string());
        process_pcd(viewer, cloud);

        // Increment iterator and loop back to beginning if end is reached
        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        // Update viewer and wait before loading next frame
        viewer->spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }

    return 0;
}