#pragma once

#define PCL_NO_PRECOMPILE

#include <vector>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"

struct PointXYZIR {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
)

namespace lidar_filtering {

struct PolarPoint {
    PointXYZIR point;
    float range;
    float azimuth;
};

class SlopeGroundFilter : public rclcpp::Node {
public:
    explicit SlopeGroundFilter(const rclcpp::NodeOptions & options);

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void organizeCloud(const pcl::PointCloud<PointXYZIR>::Ptr& input_cloud,
                      std::vector<std::vector<PolarPoint>>& rings);
    void filterGround(const std::vector<std::vector<PolarPoint>>& rings,
                     pcl::PointCloud<PointXYZIR>& out_ground,
                     pcl::PointCloud<PointXYZIR>& out_nonground);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_nonground_;

    double front_min_range_, front_max_range_;
    double rear_min_range_, rear_max_range_;
    double crop_min_y_, crop_max_y_;
    double crop_min_z_, crop_max_z_;
    double leaf_size_;

    std::string input_topic_;
    std::string ground_topic_;
    std::string nonground_topic_;
    
    double sensor_height_;
    double slope_threshold_tan_;
    int num_rings_;
    int horizontal_res_;
    double noise_floor_;
    double global_ground_tolerance_;
    double obstacle_height_thresh_;
};

} // namespace lidar_filtering