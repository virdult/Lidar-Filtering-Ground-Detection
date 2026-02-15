#include "lidar_filtering.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <pcl/filters/impl/voxel_grid.hpp>

using namespace lidar_filtering;

SlopeGroundFilter::SlopeGroundFilter(const rclcpp::NodeOptions & options)
: Node("slope_ground_filter", options) {

    this->declare_parameter("front_min_range", -15.0);
    this->declare_parameter("front_max_range", -1.35);
    this->declare_parameter("rear_min_range", 1.35);
    this->declare_parameter("rear_max_range", 15.0);
    this->declare_parameter("crop_min_y", -10.0);
    this->declare_parameter("crop_max_y", 10.0);
    this->declare_parameter("crop_min_z", -1.1);
    this->declare_parameter("crop_max_z", 2.5);
    this->declare_parameter("leaf_size", 0.05);

    this->declare_parameter("input_topic", "/ouster/points");
    this->declare_parameter("ground_topic", "/ground_points");
    this->declare_parameter("nonground_topic", "/nonground_points");
    this->declare_parameter("sensor_height", 1.05);
    this->declare_parameter("slope_threshold_deg", 10.0);
    this->declare_parameter("num_rings", 64);
    this->declare_parameter("horizontal_res", 1024);
    this->declare_parameter("noise_floor", 0.10);
    this->declare_parameter("global_ground_tolerance", 0.25);
    this->declare_parameter("obstacle_height_thresh", 0.20);

    front_min_range_ = this->get_parameter("front_min_range").as_double();
    front_max_range_ = this->get_parameter("front_max_range").as_double();
    rear_min_range_ = this->get_parameter("rear_min_range").as_double();
    rear_max_range_ = this->get_parameter("rear_max_range").as_double();
    crop_min_y_ = this->get_parameter("crop_min_y").as_double();
    crop_max_y_ = this->get_parameter("crop_max_y").as_double();
    crop_min_z_ = this->get_parameter("crop_min_z").as_double();
    crop_max_z_ = this->get_parameter("crop_max_z").as_double();
    leaf_size_ = this->get_parameter("leaf_size").as_double();

    input_topic_ = this->get_parameter("input_topic").as_string();
    ground_topic_ = this->get_parameter("ground_topic").as_string();
    nonground_topic_ = this->get_parameter("nonground_topic").as_string();
    sensor_height_ = this->get_parameter("sensor_height").as_double();
    num_rings_ = this->get_parameter("num_rings").as_int();
    horizontal_res_ = this->get_parameter("horizontal_res").as_int();
    noise_floor_ = this->get_parameter("noise_floor").as_double();
    global_ground_tolerance_ = this->get_parameter("global_ground_tolerance").as_double();
    obstacle_height_thresh_ = this->get_parameter("obstacle_height_thresh").as_double();

    double slope_deg = this->get_parameter("slope_threshold_deg").as_double();
    slope_threshold_tan_ = std::tan(slope_deg * (M_PI / 180.0));

    sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&SlopeGroundFilter::pointCloudCallback, this, std::placeholders::_1));

    pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic_, 10);
    pub_nonground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(nonground_topic_, 10);
}

void SlopeGroundFilter::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<PointXYZIR> ground_cloud;
    pcl::PointCloud<PointXYZIR> nonground_cloud;
    ground_cloud.header = cloud->header;
    nonground_cloud.header = cloud->header;

    std::vector<std::vector<PolarPoint>> rings(num_rings_);
    organizeCloud(cloud, rings);
    filterGround(rings, ground_cloud, nonground_cloud);

    sensor_msgs::msg::PointCloud2 out_g_msg, out_ng_msg;
    if (leaf_size_ > 0.0) {
        pcl::PointCloud<PointXYZIR>::Ptr ng_ptr(new pcl::PointCloud<PointXYZIR>(nonground_cloud));
        pcl::PointCloud<PointXYZIR> ng_filtered;
        pcl::VoxelGrid<PointXYZIR> voxel_grid;
        voxel_grid.setInputCloud(ng_ptr);
        voxel_grid.setLeafSize(leaf_size_, leaf_size_, 0.02);
        voxel_grid.filter(ng_filtered);
        pcl::toROSMsg(ng_filtered, out_ng_msg);
    } else {
        pcl::toROSMsg(nonground_cloud, out_ng_msg);
    }

    pcl::toROSMsg(ground_cloud, out_g_msg);
    out_g_msg.header = msg->header;
    out_ng_msg.header = msg->header;

    pub_ground_->publish(out_g_msg);
    pub_nonground_->publish(out_ng_msg);
}

void SlopeGroundFilter::organizeCloud(const pcl::PointCloud<PointXYZIR>::Ptr& input_cloud,
                                     std::vector<std::vector<PolarPoint>>& rings) {
    for(auto& r : rings) r.clear();

    for (const auto& p : input_cloud->points) {
        if (p.y < crop_min_y_ || p.y > crop_max_y_) continue;
        if (p.z > crop_max_z_ || p.z < crop_min_z_) continue; 

        bool in_front = (p.x >= front_min_range_ && p.x <= front_max_range_);
        bool in_rear = (p.x >= rear_min_range_ && p.x <= rear_max_range_);
        if (!in_front && !in_rear) continue;

        if (p.ring >= (uint16_t)num_rings_) continue;

        PolarPoint pp;
        pp.point = p;
        pp.range = std::hypot(p.x, p.y);
        pp.azimuth = std::atan2(p.y, p.x);
        if (pp.azimuth < 0) pp.azimuth += 2 * M_PI;

        rings[p.ring].push_back(pp);
    }

    for (int r = 0; r < num_rings_; ++r) {
        if (rings[r].empty()) continue;
        std::sort(rings[r].begin(), rings[r].end(),
            [](const PolarPoint& a, const PolarPoint& b) {
                return a.azimuth < b.azimuth;
            });
    }
}

void SlopeGroundFilter::filterGround(const std::vector<std::vector<PolarPoint>>& rings,
                                    pcl::PointCloud<PointXYZIR>& out_ground,
                                    pcl::PointCloud<PointXYZIR>& out_nonground) {
    
    // Track which azimuth bins have found their initial ground point
    std::vector<bool> ground_initialized(horizontal_res_, false);
    std::vector<PointXYZIR> prev_ring_ground(horizontal_res_);

    for (int r = num_rings_ - 1; r >= 0; --r) {
        if (rings[r].empty()) continue;

        for (const auto& p : rings[r]) {
            int azi_idx = static_cast<int>((p.azimuth / (2 * M_PI)) * horizontal_res_);
            azi_idx = std::max(0, std::min(azi_idx, horizontal_res_ - 1));

            bool is_ground = false;
            bool is_global_ground = std::abs(p.point.z - (-sensor_height_)) < global_ground_tolerance_;

            if (!ground_initialized[azi_idx]) {
                if (is_global_ground) {
                    is_ground = true;
                }
            } else {
                float dx = p.point.x - prev_ring_ground[azi_idx].x;
                float dy = p.point.y - prev_ring_ground[azi_idx].y;
                float dxy = std::hypot(dx, dy);
                float dz = std::abs(p.point.z - prev_ring_ground[azi_idx].z);

                if (is_global_ground) {
                    is_ground = true;
                } else if (dz < (dxy * slope_threshold_tan_)) {
                    is_ground = true;
                } else if (dz > obstacle_height_thresh_) {
                    is_ground = false;
                } else if (dz < noise_floor_ && dxy > 0.05) {
                    is_ground = true;
                }
            }

            if (is_ground) {
                out_ground.push_back(p.point);
                prev_ring_ground[azi_idx] = p.point;
                ground_initialized[azi_idx] = true;
            } else {
                out_nonground.push_back(p.point);
            }
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_filtering::SlopeGroundFilter)