#pragma region includes
#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <stdio.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/centroid.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/filters/conditional_removal.h>

#include <ground_plane_fitting.h>

#pragma endregion

#define LIDAR_HEIGTH 0.60
#define MAX_HEIGHT 0.6

using std::placeholders::_1;

class LidarNode : public rclcpp::Node
{

public:
    LidarNode();

private:
    typedef struct
    {
        float x_edge_start;
        float y_edge_start;
        float z_edge_start;
        float x_edge_end;
        float y_edge_end; 
        float z_edge_end;
        
        int min_cluster_size;
        int max_cluster_size;
        float cluster_tolerance;

        float lower_bound_estimation;
        float upper_bound_estimation;

        std::string input_topic_name;
        std::string output_topic_name;
        std::string passthrough_topic_name;
        std::string voxel_topic_name;

    } LidarConfig;
    LidarConfig lidarConfig;
    void setup();

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void ground_removal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool publish = true);

    void pass_through_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool publish = true);

    void voxel_grid_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool publish = true);

    void euclidian_clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool publish = true);

    void set_marker_properties(visualization_msgs::msg::Marker *marker, pcl::PointXYZI centre, int n, std::string frame_id);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pass_through;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_voxel;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_euclidian_cluster;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_euclidian_cluster_centroid;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_marker;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_cylinders_marker;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_non_ground;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ground;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_original =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    int j = 0;
};
