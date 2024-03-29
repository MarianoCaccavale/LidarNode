#include <lidar_node.hpp>

LidarNode::LidarNode() : Node("lidar_node")
{

    this->setup();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->lidarConfig.input_topic_name, 10, [this](sensor_msgs::msg::PointCloud2::SharedPtr msg)
        { topic_callback(msg); });
    publisher_pass_through = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->lidarConfig.passthrough_topic_name, 1);
    publisher_voxel = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->lidarConfig.voxel_topic_name, 1);
    // publisher_euclidian_cluster = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points2_euclidian_cluster", 1);
    publisher_euclidian_cluster_centroid = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->lidarConfig.output_topic_name, 1);
    publisher_marker = this->create_publisher<sensor_msgs::msg::PointCloud2>("/marker", 1);
    publisher_cylinders_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cylinders_markers", 1);

    publisher_non_ground = this->create_publisher<sensor_msgs::msg::PointCloud2>("/notgroundpoints", 1);
    publisher_ground = this->create_publisher<sensor_msgs::msg::PointCloud2>("/groundpoints", 1);
}

void LidarNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    auto startTime = std::chrono::steady_clock::now();

    // PCL still uses boost::shared_ptr internally
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // Data deserialization
    pcl::fromROSMsg(*msg, *cloud);
    pcl::fromROSMsg(*msg, *cloud_original);

    // Passthrough filter, that extract only points above ground
    pass_through_filter(cloud);

    // Remove the ground with the Fast Segmentation of 3D Point Clouds for Ground Vehicles
    ground_removal(cloud);

    // Voxel grid filter, that reduce(if necessary) the data dimensionality
    voxel_grid_filter(cloud);

    // Euclidian clustering for object detection
    euclidian_clustering(cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    RCLCPP_INFO(this->get_logger(), "Tutta la callback è terminata in: %d ms", ellapsedTime.count());

    cloud_original->clear();
}
void LidarNode::setup()
{
    // SETUP DEL YAML DEL LIDAR

    // Get the yaml file from the launch configuration
    this->declare_parameter("lidar_yaml_config_path");
    rclcpp::Parameter lidar_yaml_config_path = this->get_parameter("lidar_yaml_config_path");

    YAML::Node lidarSettingFile;
    try
    {
        lidarSettingFile = YAML::LoadFile(lidar_yaml_config_path.as_string());
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "Errore nell'apertura del file di configurazione del lidar.");
        RCLCPP_WARN(this->get_logger(), e.what());
    }

    YAML::Node lidarConfigNode = lidarSettingFile["Lidar_config"];
    //TODO TROVARE UN MODO COMODO DI STAMPARE TRAMITE LOG ROS IL CONTENUTO DEL NODO YAML
    //RCLCPP_INFO(this->get_logger(), lidarConfigNode);

    this->lidarConfig.x_edge_start = lidarConfigNode["passthrough_edges"]["x_edge_start"].as<float>();
    this->lidarConfig.y_edge_start = lidarConfigNode["passthrough_edges"]["y_edge_start"].as<float>();
    this->lidarConfig.z_edge_start = lidarConfigNode["passthrough_edges"]["z_edge_start"].as<float>();

    this->lidarConfig.x_edge_end = lidarConfigNode["passthrough_edges"]["x_edge_end"].as<float>();
    this->lidarConfig.y_edge_end = lidarConfigNode["passthrough_edges"]["y_edge_end"].as<float>();
    this->lidarConfig.z_edge_end = lidarConfigNode["passthrough_edges"]["z_edge_end"].as<float>();

    this->lidarConfig.min_cluster_size = lidarConfigNode["cluster"]["min_cluster_size"].as<int>();
    this->lidarConfig.max_cluster_size = lidarConfigNode["cluster"]["max_cluster_size"].as<int>();
    this->lidarConfig.cluster_tolerance = lidarConfigNode["cluster"]["cluster_tolerance"].as<float>();

    this->lidarConfig.lower_bound_estimation = lidarConfigNode["cone_identification"]["lower_bound_estimation"].as<float>();
    this->lidarConfig.upper_bound_estimation = lidarConfigNode["cone_identification"]["upper_bound_estimation"].as<float>();

    this->lidarConfig.input_topic_name = lidarConfigNode["topic_names"]["input_topic_name"].as<std::string>();
    this->lidarConfig.output_topic_name = lidarConfigNode["topic_names"]["output_topic_name"].as<std::string>();
    this->lidarConfig.passthrough_topic_name = lidarConfigNode["topic_names"]["passthrough_topic_name"].as<std::string>();
    this->lidarConfig.voxel_topic_name = lidarConfigNode["topic_names"]["voxel_topic_name"].as<std::string>();
}

void LidarNode::ground_removal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool publish)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    auto startTime = std::chrono::steady_clock::now();

    GroundPlaneFit fs;
    fs.mainLoop(cloud, notground_points, ground_points);

    auto endTime = std::chrono::steady_clock::now();
    auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    RCLCPP_INFO(this->get_logger(), "GPF segmentation complete in %d ms.", ellapsedTime.count());

    if (publish)
    {
        sensor_msgs::msg::PointCloud2 non_ground_points;
        pcl::toROSMsg(*notground_points, non_ground_points);
        non_ground_points.header.frame_id = cloud->header.frame_id;
        non_ground_points.header.stamp = this->get_clock()->now();
        publisher_non_ground->publish(non_ground_points);

        sensor_msgs::msg::PointCloud2 ground_points_msg;
        pcl::toROSMsg(*ground_points, ground_points_msg);
        ground_points_msg.header.frame_id = cloud->header.frame_id;
        ground_points_msg.header.stamp = this->get_clock()->now();
        publisher_ground->publish(ground_points_msg);
    }

    // Set the header so rviz doesn't trop this fucking messages
    notground_points->header = cloud->header;

    // Copy the non-groud point cloud back to the input point cloud so it can be processed further
    pcl::copyPointCloud(*notground_points, *cloud);
}

void LidarNode::pass_through_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool publish)
{

    // Create the PassThrough Filtering object
    pcl::PassThrough<pcl::PointXYZI> pass;
    // Pass the cloud to filter
    pass.setInputCloud(cloud);
    // Set the axis to filter(x, y or z)
    pass.setFilterFieldName("y");
    // Set the limits of the filter, it's an interval like [x, y]
    pass.setFilterLimits(this->lidarConfig.y_edge_start, this->lidarConfig.y_edge_end);

    // Apply the filter
    pass.filter(*cloud);

    // Pass the cloud to filter
    pass.setInputCloud(cloud);
    // Set the axis to filter(x, y or z)
    pass.setFilterFieldName("x");
    // Set the limits of the filter, it's an interval like [x, y]
    pass.setFilterLimits(this->lidarConfig.x_edge_start, this->lidarConfig.x_edge_end);
    // Apply the filter
    pass.filter(*cloud);

    // Pass the cloud to filter
    pass.setInputCloud(cloud);
    // Set the axis to filter(x, y or z)
    pass.setFilterFieldName("z");
    // Set the limits of the filter, it's an interval like [x, y]
    pass.setFilterLimits(this->lidarConfig.z_edge_start, this->lidarConfig.z_edge_end);
    // Apply the filter
    pass.filter(*cloud);

    sensor_msgs::msg::PointCloud2 filtered_cloud_points;
    pcl::toROSMsg(*cloud, filtered_cloud_points);

    // Publish the filtered points(converted in sensor_msgs::msg::PointCloud2) back to a topic
    // so that Rviz can render them
    if (true)
    {
        publisher_pass_through->publish(filtered_cloud_points);
    }
}

void LidarNode::voxel_grid_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool publish)
{

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    // Pass the cloud to filter
    sor.setInputCloud(cloud);
    // Set the parameters for the downsampling
    sor.setLeafSize(0.05f, 0.05f, 0.01f);
    // Apply the filter
    sor.filter(*cloud);

    sensor_msgs::msg::PointCloud2 filtered_cloud_points;
    pcl::toROSMsg(*cloud, filtered_cloud_points);

    // Publish the filtered points(converted in sensor_msgs::msg::PointCloud2) back to a topic
    // so that Rviz can render them
    if (true)
    {
        publisher_voxel->publish(filtered_cloud_points);
    }
}

void LidarNode::euclidian_clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool publish)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    visualization_msgs::msg::MarkerArray marker_array_msg;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cylinders_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(this->lidarConfig.cluster_tolerance); // 10cm   //Mariano.py può cambiare
    ec.setMinClusterSize(this->lidarConfig.min_cluster_size);
    ec.setMaxClusterSize(this->lidarConfig.max_cluster_size);
    ec.setSearchMethod(tree);

    pcl::PointCloud<pcl::PointXYZI>::Ptr centroids_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    centroids_cloud->height = 1;
    centroids_cloud->is_dense = true;

    std::vector<pcl::PointXYZI> centroids_array;

    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto &cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI centroid;
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        }

        // Compute the centroid in order to use it to extract the (hopefully) cone cloud
        pcl::computeCentroid(*cloud_cluster, centroid);
        RCLCPP_INFO(this->get_logger(), "++Centroide: %f - %f - %f", centroid.x, centroid.y, centroid.z);

        pcl::ConditionAnd<pcl::PointXYZI>::Ptr cyl_cond(new pcl::ConditionAnd<pcl::PointXYZI>());

        Eigen::Matrix3f cylinderMatrix;

        cylinderMatrix << 1.0, 0, 0,
            0, 1.0, 0,
            0, 0, 0;

        Eigen::Vector3f cylinderPosition;
        cylinderPosition << -centroid.x, -centroid.y, 0;

        double radius = 0.15;
        float cylinderScalar = -(radius * radius) + centroid.x * centroid.x + centroid.y * centroid.y;

        pcl::TfQuadraticXYZComparison<pcl::PointXYZI>::Ptr cyl_comp(new pcl::TfQuadraticXYZComparison<pcl::PointXYZI>(pcl::ComparisonOps::LE, cylinderMatrix, cylinderPosition, cylinderScalar));
        cyl_cond->addComparison(cyl_comp);

        // build and apply filter
        pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
        condrem.setCondition(cyl_cond);
        condrem.setInputCloud(cloud_original);
        condrem.setKeepOrganized(false);
        condrem.filter(*cylinder_cloud);

        // Controllo se il numero di punti in questo ostacolo è circa uguale al numero di punti che avrebbe un cono alla distanza
        // a cui si trova l'ostacolo(la distanza dell'ostacolo è la x del centroide dello stesso).
        // Se l'ostacolo ha troppi o tropi pochi punti, lo scarto, altrimenti è un cono e aggiungo il centroide alla lista

        double distance = sqrt(centroid.x * centroid.x + centroid.y * centroid.y + centroid.z * centroid.z);
        static double rv = 0.2 * M_PI / 180.0; // angular resolution vertical
        static double rh = 0.2 * M_PI / 180.0; // angular resolution horizontal
        static double hc = 0.325;              // cone height
        static double wc = 0.228;              // cone width

        double estimeted_number_of_points = 0.5 * hc / (2 * distance * tan(rv / 2)) * wc / (2 * distance * tan(rh / 2));

        /* std::cout << "Centroid.x =  [" << centroid.x << "] (distanza asse x) " << std::endl;
        std::cout << "Centroid.y =  [" << centroid.y << "] (distanza asse y) " << std::endl;
        std::cout << "Centroid.z =  [" << centroid.z << "] (distanza asse z) " << std::endl
                  << std::endl; */

        // If the cluster is inside the range [80%, 120%] of the estimated cone at distance D, then it is a cone
        if ((cylinder_cloud->size() >= estimeted_number_of_points * this->lidarConfig.lower_bound_estimation) && (cylinder_cloud->size() <= estimeted_number_of_points * this->lidarConfig.upper_bound_estimation))
        {

            RCLCPP_INFO(this->get_logger(), "HO TROVATO UN CONO!");

            // join each cloud cluster into one combined cluster (visualisation)
            *cylinders_cloud += *cylinder_cloud;

            // If the obstacles has the same amount of point of a cone, I add the centroid
            centroids_cloud->push_back(centroid);
            centroids_cloud->width = centroids_cloud->size();
            centroids_array.push_back(centroid);
        }

        RCLCPP_INFO(this->get_logger(), "++Cluster a distanza %f con %d punti ha n_punti stimato: %f estimeted_number_of_points", distance, cylinder_cloud->size());

        cylinder_cloud->clear();
        cloud_cluster->clear();
    }

    marker_array_msg.markers.resize(centroids_array.size());
    for (size_t i = 0; i < centroids_array.size(); ++i)
    {
        set_marker_properties(&marker_array_msg.markers[i], centroids_array[i], i, cloud->header.frame_id);
    }

    // Publish the centroids(center of obstacles, not only cones)
    sensor_msgs::msg::PointCloud2 centroids_cloud_msg;
    pcl::toROSMsg(*centroids_cloud, centroids_cloud_msg);
    centroids_cloud_msg.header.frame_id = cloud->header.frame_id;
    centroids_cloud_msg.header.stamp = this->get_clock()->now();
    publisher_euclidian_cluster_centroid->publish(centroids_cloud_msg);

    // Publish the cylinders point cloud, hopefully with only the cones inside
    sensor_msgs::msg::PointCloud2 cylinder_cloud_msg;
    pcl::toROSMsg(*cylinders_cloud, cylinder_cloud_msg);
    cylinder_cloud_msg.header.frame_id = cloud->header.frame_id;
    cylinder_cloud_msg.header.stamp = this->get_clock()->now();
    publisher_marker->publish(cylinder_cloud_msg);

    // Publish the cylinders markers, to see where the algorithm search the cones
    publisher_cylinders_marker->publish(marker_array_msg);

    centroids_cloud->clear();
    cylinders_cloud->clear();
    cluster_indices.clear();
}

void LidarNode::set_marker_properties(visualization_msgs::msg::Marker *marker, pcl::PointXYZI centre, int n, std::string frame_id)
{
    marker->header.frame_id = frame_id;
    // marker->header.stamp = Node.now();
    marker->ns = "my_namespace";
    marker->id = n;
    marker->type = visualization_msgs::msg::Marker::CYLINDER;
    marker->action = visualization_msgs::msg::Marker::ADD;

    marker->pose.position.x = centre.x;
    marker->pose.position.y = centre.y;
    marker->pose.position.z = centre.z;

    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;

    marker->scale.x = 0.30;
    marker->scale.y = 0.30;
    marker->scale.z = 0.7;

    // alpha and RGB settings
    marker->color.a = 0.4;
    marker->color.r = 0.0;
    marker->color.g = 1.0;
    marker->color.b = 0.0;

    // marker->lifetime = ros::Duration(0.1);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarNode>());
    rclcpp::shutdown();
    return 0;
}