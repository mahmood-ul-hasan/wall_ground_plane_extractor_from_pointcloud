#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h> // Include this header for NormalEstimation 
#include <pcl/point_cloud.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>


#include <ros/ros.h>
#include <ros/package.h> // Include the ROS package header

typedef pcl::PointXYZI PointT;

pcl::visualization::PCLVisualizer viewer("All Planar Components Point Clouds");
       int viewport1, viewport2;



// Function to write data to 'wall_seg_n.txt' file
void writeInlierIndicesToFile(const std::vector<int>& sIndicesSorted,
                         const std::vector<int>& iIndicesSorted,
                         const pcl::ModelCoefficients& model1, const std::stringstream& fileName) {
    // Open the file
    std::ofstream fileID(fileName.str());

    // Write the data to the file
    fileID << "Sample Indices\n";
    fileID << "Indices Size:\n";
    fileID << sIndicesSorted.size() << "\n";
    fileID << "Data:\n";
    for (size_t i = 0; i < sIndicesSorted.size(); i++) {
        fileID << sIndicesSorted[i] << "\n";
    }
    fileID << "Coefficient of the plane:\n";
    fileID << model1.values[0] << " " << model1.values[1] << " " << model1.values[2] << " " << model1.values[3] << "\n";
    fileID << "Inliers Indices\n";
    fileID << "Indices Size:\n";
    fileID << iIndicesSorted.size() << "\n";
    fileID << "Data:\n";
    for (size_t i = 0; i < iIndicesSorted.size(); i++) {
        fileID << iIndicesSorted[i] << "\n";
    }

    // Close the file
    fileID.close();
}


pcl::PointCloud<PointT>::Ptr extractLargestCluster(const pcl::PointCloud<PointT>::Ptr &input_cloud, std::vector<int> &largest_cluster_indices, float cluster_tolerance)
{
  
    // Create the KdTree object
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(input_cloud);

    // Euclidean Cluster Extraction parameters
    int min_cluster_size = 100;     // Minimum number of points that a cluster should have
    int max_cluster_size = 2500000;   // Maximum number of points that a cluster should have

    // Euclidean Cluster Extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);


    // std::cout << "Input cloud size: " << input_cloud->size();
    // std::cout << " --- Number of cluster indices found: " << cluster_indices.size() << std::endl;
    // Find the largest cluster
    int largest_cluster_index = 0;
    size_t largest_cluster_size = 0;
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        if (cluster_indices[i].indices.size() > largest_cluster_size)
        {
            largest_cluster_size = cluster_indices[i].indices.size();
            largest_cluster_index = i;
        }
    }

    // Update the largest cluster indices in the argument
    largest_cluster_indices = cluster_indices[largest_cluster_index].indices;


    // Extract the largest cluster from the input point cloud
    pcl::PointCloud<PointT>::Ptr largest_cluster(new pcl::PointCloud<PointT>);
    for (const auto &index : largest_cluster_indices)
    {
        largest_cluster->points.push_back(input_cloud->points[index]);
    }
    largest_cluster->width = static_cast<uint32_t>(largest_cluster->points.size());
    largest_cluster->height = 1;
    largest_cluster->is_dense = true;

    return largest_cluster;
}



void extract_wall_planes(pcl::PointCloud<PointT>::Ptr &cloud_filtered,
                        std::string file_directory,
                         float wall_distance_threshold,
                         float wall_angle_degree,
                         float min_wall_plane_size)
{

pcl::PointCloud<PointT>::Ptr cloud_w (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
pcl::ExtractIndices<PointT> extract;

int i = 1, nr_points = (int) cloud_filtered->size ();
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

// Visualization of the cloud_filtered point cloud	
pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud_filtered, 255, 255, 255);	


pcl::visualization::PCLVisualizer viewer_wall_all("Wall Planar Components");	
// Create 10 viewports
int num_rows = 2; // Number of rows of viewports
int num_cols = 3; // Number of columns of viewports
int cycle_interval = num_rows*num_cols;

for (int row = 0; row < num_rows; ++row) {
    for (int col = 0; col < num_cols; ++col) {
        int viewport_num = row * num_cols + col + 1; // Calculate the viewport number
        double x_min = static_cast<double>(col) / num_cols;
        double x_max = static_cast<double>(col + 1) / num_cols;
        double y_min = 1.0 - static_cast<double>(row + 1) / num_rows;
        double y_max = 1.0 - static_cast<double>(row) / num_rows;
        viewer_wall_all.createViewPort(x_min, y_min, x_max, y_max, viewport_num);
    }
}

std::vector<pcl::visualization::PointCloudColorHandlerCustom<PointT>> colors_wall;
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 0, 0, 255));      // Blue
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 255, 255, 0));    // Yellow
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 128, 0, 128));    // Purple
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 255, 165, 0));    // Orange
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 0, 128, 128)); // 5. Teal
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 0, 0, 139)); // 6. Dark Blue
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 220, 20, 60));// 7. Crimson 
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 50, 205, 50)); // 8. Lime Green
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 135, 206, 235)); // 9. Sky Blu
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 230, 230, 250)); // 10. Lavender



Eigen::Vector3f wall_axis;
float wall_angle =  wall_angle_degree * M_PI / 180.0;; // Set the desired angle in degrees (e.g., 40.0)

// Create the segmentation object
pcl::SACSegmentation<PointT> seg_wall;
seg_wall.setOptimizeCoefficients (true);
// seg_wall.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
// seg_wall.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
seg_wall.setModelType (pcl::SACMODEL_PLANE);
seg_wall.setMethodType (pcl::SAC_RANSAC);
seg_wall.setMaxIterations (1000);
seg_wall.setDistanceThreshold (wall_distance_threshold);
seg_wall.setEpsAngle(wall_angle);

// Segment the wall planar component from the remaining cloud
// While 30% of the original cloud is still there to extract the wall plane
while (cloud_filtered->size () > 0.01 * nr_points)
{

    bool found_plane = false;

// Check for walls in the X-axis
    wall_axis << 1.0, 0.0, 0.0; // Wall plane; plane perpendicular to the X-axis (assuming walls are parallel to the YZ-plane)
    std::cerr << "Segment the wall plane in the X-axis" << std::endl;
    seg_wall.setAxis(wall_axis);
    seg_wall.setInputCloud(cloud_filtered);
    seg_wall.segment(*inliers, *coefficients);

    // If the found plane in the X-axis is not large enough, check the Y-axis
  if (inliers->indices.size() < min_wall_plane_size) {
      std::cerr << "Segment the wall plane in the Y-axis" << std::endl;

      // Segment the wall planar component from the remaining cloud in the Y-axis
      wall_axis << 0.0, 1.0, 0.0; // Wall plane; plane perpendicular to the Y-axis (assuming walls are parallel to the XZ-plane)
      seg_wall.setAxis(wall_axis);
      seg_wall.setInputCloud(cloud_filtered);
      seg_wall.segment(*inliers, *coefficients);
  }

  if (inliers->indices.size () <= min_wall_plane_size)
    {
    std::cerr << "The size of estimated wall plane is less than given parameter min_wall_plane_size so stopping extracting small wall planes" << std::endl;
    break;
    }

  // Extract the inliers
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_w);
  
  std::vector<int> wall_inliers;

   for (size_t i = 0; i < cloud_w->size(); ++i) {
        const PointT& point = cloud_w->at(i);
        int intensity = static_cast<int>(point.intensity);
        wall_inliers.push_back(intensity);
    }
  
  pcl::PointCloud<PointT>::Ptr cloud_w_cluster;
  cloud_w_cluster = cloud_w;

  std::cout << "wall planar: " << i << " has " << inliers->indices.size () << " data points." << std::endl;
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << file_directory << "wall_seg_cloud" << i << ".pcd";
  writer.write<PointT> (ss.str (), *cloud_w, false);
  std::stringstream wall_indices_file_name;
  wall_indices_file_name << file_directory  << "wall_seg" << i << ".txt";
// Function to write data to 'wall_seg_n.txt' file
   writeInlierIndicesToFile(wall_inliers, wall_inliers, *coefficients, wall_indices_file_name);

  // Visualization of the cloud_w point cloud	
  viewer_wall_all.addPointCloud<PointT>(cloud_w_cluster, colors_wall[i-1 % colors_wall.size()], ss.str()+ "no_cluster_all", i );
  viewer.addPointCloud<PointT>(cloud_w_cluster, colors_wall[i-1 % colors_wall.size()], ss.str(),viewport2);



// pcl::visualization::PCLVisualizer viewer_one_plane("viewer_one_plane");
// viewer_one_plane.setBackgroundColor(0, 0, 0); // Set background color to black
// viewer_one_plane.addPointCloud<PointT>(cloud_w_cluster, single_color, "combined_extracted_wall_plane1");
// viewer_one_plane.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "combined_extracted_wall_plane1");



  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_f);
  cloud_filtered.swap (cloud_f);
  i++;
}
}



void extract_wall_planes_auto_extracted_wall_axes(pcl::PointCloud<PointT>::Ptr &cloud_filtered,
                           std::string  file_directory,
                         float wall_distance_threshold,
                         float wall_angle_degree,
                         float min_wall_plane_size)
{
pcl::PointCloud<PointT>::Ptr cloud_w (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
pcl::ExtractIndices<PointT> extract;

int i = 1, nr_points = (int) cloud_filtered->size ();
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

// Visualization of the cloud_filtered point cloud	
pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud_filtered, 255, 255, 255);	
pcl::visualization::PCLVisualizer viewer_wall_all("Wall Planar Components Point Clouds");	
viewer_wall_all.setBackgroundColor(0, 0, 0);

// Create 10 viewports
int num_rows = 2; // Number of rows of viewports
int num_cols = 5; // Number of columns of viewports
int cycle_interval = num_rows*num_cols;

for (int row = 0; row < num_rows; ++row) {
    for (int col = 0; col < num_cols; ++col) {
        int viewport_num = row * num_cols + col + 1; // Calculate the viewport number
        double x_min = static_cast<double>(col) / num_cols;
        double x_max = static_cast<double>(col + 1) / num_cols;
        double y_min = 1.0 - static_cast<double>(row + 1) / num_rows;
        double y_max = 1.0 - static_cast<double>(row) / num_rows;
        viewer_wall_all.createViewPort(x_min, y_min, x_max, y_max, viewport_num);
    }
}

std::vector<pcl::visualization::PointCloudColorHandlerCustom<PointT>> colors_wall;
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 0, 0, 255));      // Blue
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 255, 255, 0));    // Yellow
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 128, 0, 128));    // Purple
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 255, 165, 0));    // Orange
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 0, 128, 128)); // 5. Teal
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 0, 0, 139)); // 6. Dark Blue
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 220, 20, 60));// 7. Crimson 
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 50, 205, 50)); // 8. Lime Green
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 135, 206, 235)); // 9. Sky Blu
colors_wall.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_w, 230, 230, 250)); // 10. Lavender




// Create a PCLVisualizer object
pcl::visualization::PCLVisualizer viewer_dominant("Dominant Plane Viewer");
// Set the background color to black (optional)
viewer_dominant.setBackgroundColor(0.0, 0.0, 0.0);
viewer_dominant.addPointCloud<PointT>(cloud_filtered, single_color, "cloud_filtered");	



Eigen::Vector3f wall_axis;
float wall_angle =  wall_angle_degree * M_PI / 180.0;; // Set the desired angle in degrees (e.g., 40.0)

// Create the segmentation object
pcl::SACSegmentation<PointT> seg_wall;
seg_wall.setOptimizeCoefficients (true);
// seg_wall.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
// seg_wall.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
seg_wall.setModelType (pcl::SACMODEL_PLANE);
seg_wall.setMethodType (pcl::SAC_RANSAC);
seg_wall.setMaxIterations (1000);
seg_wall.setDistanceThreshold (wall_distance_threshold);
seg_wall.setEpsAngle(wall_angle);

// Segment the wall planar component from the remaining cloud
// While 30% of the original cloud is still there to extract the wall plane
while (cloud_filtered->size () > 0.01 * nr_points)
{

bool found_plane = false;


// Step 1: Compute Normals
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
pcl::NormalEstimation<PointT, pcl::Normal> ne;
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
ne.setInputCloud(cloud_filtered);
ne.setSearchMethod(tree);
ne.setKSearch(200); // Adjust this value based on your point cloud density
ne.compute(*normals);

// Step 2: Cluster planes based on normal vectors using Region Growing
pcl::RegionGrowing<PointT, pcl::Normal> reg;
reg.setMinClusterSize(100);  // Adjust this value based on the minimum number of points in a plane
reg.setMaxClusterSize(10000); // Adjust this value based on the maximum number of points in a plane
reg.setSearchMethod(tree); // Use the same search tree as in the normal estimation
reg.setInputCloud(cloud_filtered);
reg.setInputNormals(normals); // Provide the computed normals to the region growing algorithm

std::vector<pcl::PointIndices> cluster_indices;
reg.extract(cluster_indices);

// Step 3: Identify the dominant plane
std::size_t max_cluster_size = 0;
std::size_t dominant_plane_index = 0;

for (std::size_t i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.size() > max_cluster_size) {
        max_cluster_size = cluster_indices[i].indices.size();
        dominant_plane_index = i;
    }
}

Eigen::Vector3f wall_axis;
   
if (dominant_plane_index < cluster_indices.size()) {
    const pcl::PointIndices& dominant_plane_indices = cluster_indices[dominant_plane_index];
    // Calculate the average normal of the dominant plane
    Eigen::Vector3f average_normal(0.0, 0.0, 0.0);
    for (const auto& index : dominant_plane_indices.indices) {
        average_normal += Eigen::Vector3f(normals->at(index).normal_x,
                                          normals->at(index).normal_y,
                                          normals->at(index).normal_z);
    }
    average_normal /= static_cast<float>(dominant_plane_indices.indices.size());
    average_normal.normalize();
    wall_axis = average_normal;
    std::cout << "wall_axis: " << wall_axis << std::endl;
} else {
    std::cerr << "Failed to identify the dominant plane. Using a default wall_axis." << std::endl;
    // Set a default wall_axis (e.g., parallel to X-axis)
    wall_axis << 1.0, 0.0, 0.0;
}



// Step 4: Visualize the dominant plane
pcl::PointCloud<PointT>::Ptr dominant_plane_cloud(new pcl::PointCloud<PointT>);
pcl::copyPointCloud(*cloud_filtered, cluster_indices[dominant_plane_index].indices, *dominant_plane_cloud);

//-------------------------------------------



// Check for walls in the X-axis
    // wall_axis << 1.0, 0.0, 0.0; // Wall plane; plane perpendicular to the X-axis (assuming walls are parallel to the YZ-plane)
    std::cerr << "Segment the wall plane in the X-axis" << std::endl;
    // seg_wall.setAxis(wall_axis);
    seg_wall.setInputCloud(cloud_filtered);
    seg_wall.segment(*inliers, *coefficients);


  if (inliers->indices.size () <= min_wall_plane_size)
    {
    std::cerr << "The size of estimated wall plane is less than given parameter min_wall_plane_size so stopping extracting small wall planes" << std::endl;
    break;
    }

  // Extract the inliers
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_w);
  

  std::vector<int> wall_inliers;

   for (size_t i = 0; i < cloud_w->size(); ++i) {
        const PointT& point = cloud_w->at(i);
        int intensity = static_cast<int>(point.intensity);
        wall_inliers.push_back(intensity);
    }

  pcl::PointCloud<PointT>::Ptr cloud_w_cluster;
  cloud_w_cluster = cloud_w;

  std::cout << "wall planar: " << i << " has " << cloud_w_cluster->width * cloud_w_cluster->height << " data points." << std::endl;

  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << file_directory << "wall_seg_cloud" << i << ".pcd";
  writer.write<PointT> (ss.str (), *cloud_w, false);
  

  std::stringstream wall_indices_file_name;
  wall_indices_file_name << file_directory << "wall_seg" << i << ".txt";
// Function to write data to 'wall_seg_n.txt' file
   writeInlierIndicesToFile(wall_inliers, wall_inliers, *coefficients, wall_indices_file_name);



  // Visualization of the cloud_w point cloud	
  viewer_wall_all.addPointCloud<PointT>(cloud_w_cluster, colors_wall[i-1 % colors_wall.size()], ss.str()+ "no_cluster_all", i-1 );
  viewer.addPointCloud<PointT>(cloud_w_cluster, colors_wall[i-1 % colors_wall.size()], ss.str(),viewport2);

  // Add the dominant plane cloud to the viewer with a green color
  viewer_dominant.addPointCloud<PointT>(dominant_plane_cloud, colors_wall[i-1 % colors_wall.size()], ss.str());


  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_f);
  cloud_filtered.swap (cloud_f);
  i++;
}
}


//load the pcd file and store the point cloud data to the cloud variable
void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud)
{
  std::cout<< "file" << file << std::endl;
  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read the pcd file\n");
    std::exit(-1);
  }
  std::cout << "Loaded "
            << cloud->width << "x" << cloud->height
            << " data points from the pcd file"
            << std::endl;
}


int main (int argc, char** argv)
{
 
 ros::init(argc, argv, "plane_extraction");
  ros::NodeHandle nh;

  std::string file_directory;  // Declare the file_directory variable


  double ground_angle_degree;
  double ground_distance_threshold;
  double min_ground_plane_size;
  double ground_plane_cluster_Tolerance;
  double wall_distance_threshold;
  double wall_angle_degree;
  double min_wall_plane_size;
  bool ground_plane_cluster_extraction;



  // // Retrieve parameters from the ROS parameter server
  nh.param<double>("ground_angle_degree", ground_angle_degree, 0.0);
  nh.param<double>("ground_distance_threshold", ground_distance_threshold, 0.0);
  nh.param<double>("min_ground_plane_size", min_ground_plane_size, 0.0);
  nh.param<double>("ground_plane_cluster_Tolerance", ground_plane_cluster_Tolerance, 0.0);  // Check parameter name
  nh.param<double>("wall_distance_threshold", wall_distance_threshold, 0.0);
  nh.param<double>("wall_angle_degree", wall_angle_degree, 0.0);
  nh.param<double>("min_wall_plane_size", min_wall_plane_size, 0.0);
    nh.param<bool>("ground_plane_cluster_extraction", ground_plane_cluster_extraction, false);

  
ROS_INFO("Checking if the parameter exists: %d", nh.hasParam("file_directory"));


 // Retrieve the file_directory parameter from the ROS parameter server
  if (nh.getParam("file_directory", file_directory)) {
    ROS_INFO("file_directory: %s", file_directory.c_str());
  } else {
    ROS_ERROR("Failed to retrieve file_directory parameter from the parameter server.");
  }


  pcl::PCLPointCloud2::Ptr cloud_input (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>), cloud_input_pcl (new pcl::PointCloud<PointT>), cloud_f (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_g (new pcl::PointCloud<PointT>);


  std::string pcd_file = file_directory + "original_data.pcd";
  load_pcd_file(pcd_file, cloud_input_pcl);


  // Populate the point cloud and save the index value in the intensity field of cloud_filtered_index
  for (size_t i = 0; i < cloud_input_pcl->size(); ++i) {
      PointT point = cloud_input_pcl->at(i); // Copy the point from cloud_filtered
      point.intensity = static_cast<float>(i); // Save the index value as intensity
      cloud_filtered->push_back(point); // Add the point to cloud_filtered_index
  }
   viewer.setBackgroundColor(0, 0, 0);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, viewport1);  // Left-top
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, viewport2); // Center-top

pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud_g, 255, 255, 255);

  viewer.addPointCloud<PointT>(cloud_filtered, single_color, "input_cloud_filtered", viewport1);	


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  Eigen::Vector3f ground_axis;
  ground_axis << 0.0, 0.0, 1.0; // Ground plane; plane perpendicular to the Z-axis
  float ground_angle = ground_angle_degree * M_PI / 180.0;; // Set the desired angle in degrees (e.g., 40.0)

  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  // seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setEpsAngle(ground_angle);
  seg.setDistanceThreshold (ground_distance_threshold);


  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  int i = 1, nr_points = (int) cloud_filtered->size ();


  // List of different colors
  std::vector<pcl::visualization::PointCloudColorHandlerCustom<PointT>> colors;
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_g, 255, 0, 0));      // Red
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_g, 0, 255, 0));      // Green
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_g, 0, 128, 0));       // Dark Green
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_g, 255, 0, 128));     // Pink
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_g, 255, 0, 255));// 2. Magenta
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_g, 154, 205, 50));// 3. Yellow-Green
  colors.push_back(pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_g, 255, 215, 0)); // 4. Gold
  


  pcl::visualization::PCLVisualizer viewer_ground_plane("Seprate ground Planar Components");	
  viewer_ground_plane.setBackgroundColor(0, 0, 0);
// Create 10 viewports
int num_rows = 2; // Number of rows of viewports
int num_cols = 3; // Number of columns of viewports
int cycle_interval = num_rows*num_cols;

for (int row = 0; row < num_rows; ++row) {
    for (int col = 0; col < num_cols; ++col) {
        int viewport_num = row * num_cols + col + 1; // Calculate the viewport number
        double x_min = static_cast<double>(col) / num_cols;
        double x_max = static_cast<double>(col + 1) / num_cols;
        double y_min = 1.0 - static_cast<double>(row + 1) / num_rows;
        double y_max = 1.0 - static_cast<double>(row) / num_rows;
        viewer_ground_plane.createViewPort(x_min, y_min, x_max, y_max, viewport_num);
    }
}



  // While 10% of the original cloud is still there  to extract the ground plane
  while (cloud_filtered->size () > 0.1 * nr_points)
  {
    
    // Segment the ground planar component from the remaining cloud
    seg.setAxis(ground_axis);
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    // Convert pcl::PointIndices::Ptr to std::vector<int>
    std::vector<int> ground_inliers ;


    if (inliers->indices.size () < min_ground_plane_size)
    {
      std::cerr << "Estimated ground plane size is less than 'min_ground_plane_size' so stopping planes extraction" << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_g);


    std::vector<int> ground_largest_cluster_indices;

 pcl::PointCloud<PointT>::Ptr cloud_g_cluster;
 if (ground_plane_cluster_extraction == true){
    // extract Largest Cluster
  cloud_g_cluster = extractLargestCluster(cloud_g, ground_largest_cluster_indices, ground_plane_cluster_Tolerance);
}
  else if(ground_plane_cluster_extraction == false){
    cloud_g_cluster = cloud_g;
   }


  for (size_t i = 0; i < cloud_g_cluster->size(); ++i) {
        const PointT& point = cloud_g_cluster->at(i);
        int intensity = static_cast<int>(point.intensity);
        ground_inliers.push_back(intensity);
    }


    // Save the extracted Cluster
    std::cout << "Ground planar: " << i << " has " << inliers->indices.size ()  << " data points." << std::endl;
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << file_directory << "ground_seg_cloud" << i << ".pcd";
    writer.write<PointT> (ss.str (), *cloud_g_cluster, false);
    std::stringstream ground_indices_file_name;
    ground_indices_file_name << file_directory <<  "ground_seg" << i << ".txt";
    // Function to write data to 'wall_seg_n.txt' file

     if (ground_plane_cluster_extraction == true){
    writeInlierIndicesToFile(ground_inliers, ground_inliers, *coefficients, ground_indices_file_name);
     }
       else if(ground_plane_cluster_extraction == false){
    writeInlierIndicesToFile(ground_inliers, ground_inliers, *coefficients, ground_indices_file_name);
       }


    // Visualization of the cloud_g point cloud	
    viewer.addPointCloud<PointT>(cloud_g_cluster, colors[i-1 % colors.size()], ss.str(), viewport1);

    viewer_ground_plane.addPointCloud<PointT>(cloud_g_cluster, colors[i-1 % colors.size()], ss.str(),  i);
    viewer_ground_plane.spinOnce(100, true);


    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;


  }

    std::cout << "extracting ground plane finished " << i << std::endl;
    std::cout << "==================================================================================" << std::endl;

    viewer.addPointCloud<PointT>(cloud_filtered, single_color, "cloud_filtered", viewport2);

// extracting walls plane by different methods
extract_wall_planes(cloud_filtered, file_directory, wall_distance_threshold, wall_angle_degree,  min_wall_plane_size);
// extract_wall_planes_auto_extracted_wall_axes(cloud_filtered, file_directory, wall_distance_threshold, wall_angle_degree,  min_wall_plane_size);

viewer.spin();
return (0);
}
