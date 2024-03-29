#ifndef _PLANE_FIT_H_
#define _PLANE_FIT_H_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

typedef struct {
    Eigen::MatrixXf normal_n;
    double d = 0.;
} model_t;

class GroundPlaneFit : public rclcpp::Node{

public:
    GroundPlaneFit();
    ~GroundPlaneFit();

    void mainLoop(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points,
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points);

private:
    model_t estimatePlane(const pcl::PointCloud<pcl::PointXYZI>& seed_points);
    void extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points);

    int num_seg_;
    int num_iter_;
    int num_lpr_;

    float th_seeds_;
    float th_dist_;

    float sensor_height_;
};

#endif
