typedef pcl::PointXYZI PointT;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<key_index.hpp>

pcl::PointCloud<PointT>::Ptr generatePlaneFromCoefficients(const Eigen::VectorXf& coefficients, const pcl::PointCloud<PointT>::Ptr input_cloud);
void visualize_some_2d_scan_lines_in_3d_pointcloud( pcl::PointCloud<PointT>::Ptr cloud, key_index filterred_idx);
void generate_all_plane_from_Ccoefficients(Eigen::VectorXf coeffs_wall1, pcl::PointCloud<PointT>::Ptr plane_points_wall1, Eigen::VectorXf coeffs_wall2, pcl::PointCloud<PointT>::Ptr plane_points_wall2, Eigen::VectorXf coeffs_wall3, pcl::PointCloud<PointT>::Ptr plane_points_wall3,
Eigen::VectorXf coeffs_wall4, pcl::PointCloud<PointT>::Ptr plane_points_wall4, Eigen::VectorXf coeffs_wall5, pcl::PointCloud<PointT>::Ptr plane_points_wall5, Eigen::VectorXf coeffs_ground1, pcl::PointCloud<PointT>::Ptr plane_points_ground1, 
Eigen::VectorXf coeffs_ground2, pcl::PointCloud<PointT>::Ptr plane_points_ground2,Eigen::VectorXf coeffs_ground3, pcl::PointCloud<PointT>::Ptr plane_points_ground3,Eigen::VectorXf coeffs_ground4, pcl::PointCloud<PointT>::Ptr plane_points_ground4,
Eigen::VectorXf coeffs_ground5, pcl::PointCloud<PointT>::Ptr plane_points_ground5);

void plot_inliers_idx_of_all_planes(std::vector<int> inliers_idx_ground1, std::vector<int> inliers_idx_ground2, std::vector<int> inliers_idx_ground3, std::vector<int> inliers_idx_ground4, std::vector<int> inliers_idx_ground5, std::vector<int> inlier_index_wall1, std::vector<int> inlier_index_wall2, std::vector<int> inlier_index_wall3, std::vector<int> inlier_index_wall4, std::vector<int> inlier_index_wall5 );
void plot_index_mapping_of_one_plane(key_index filterred_idx, key_index filterred_idx_wall1,  key_index plane_idx_wall1, std::vector<int> inlier_index_wall1, std::vector<int>  roi_index_wall1);
void plot_filtered_idx_of_all_planes(key_index filterred_idx, key_index filterred_idx_wall1, key_index filterred_idx_wall2, key_index filterred_idx_wall3, key_index filterred_idx_wall4, key_index filterred_idx_ground1, key_index filterred_idx_ground2, key_index filterred_idx_ground3, key_index filterred_idx_ground4);
void plot_plane_idx_of_all_planes(key_index filterred_idx, key_index plane_idx_wall1, key_index plane_idx_wall2, key_index plane_idx_wall3, key_index plane_idx_wall4, key_index plane_idx_ground1, key_index plane_idx_ground2, key_index plane_idx_ground3, key_index plane_idx_ground4);
void verify_visualize_plane_frame_idx(pcl::PointCloud<PointT>::Ptr cloud, int number_of_wall_planes, key_index plane_idx_wall1, key_index plane_idx_wall2, key_index plane_idx_wall3, key_index plane_idx_wall4, key_index plane_idx_wall5, std::vector<int> inlier_index_wall1, std::vector<int> inlier_index_wall2,std::vector<int> inlier_index_wall3,std::vector<int> inlier_index_wall4,std::vector<int> inlier_index_wall5);
void verify_visualize_one_plane_frame_idx(pcl::PointCloud<PointT>::Ptr cloud, key_index plane_idx_ground1, std::vector<int> inliers_idx_ground1);

