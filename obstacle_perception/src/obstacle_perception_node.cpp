#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Geometry>  // 包含Eigen库的几何模块，用于处理变换相关操作

// 定义用于表示单线雷达数据的结构体（这里简单示意，可以根据实际需求调整结构内容）
struct SingleLineLaserData {
    std::vector<float> ranges;
    std::vector<float> intensities;
};

// 定义用于表示栅格地图的结构体（简单示意，可按需扩展完善）
struct GridMap {
    int width;
    int height;
    float resolution;
    std::vector<bool> grid_data;
};

class ObstaclePerceptionNode {
public:
    ObstaclePerceptionNode(ros::NodeHandle& nh);
    ~ObstaclePerceptionNode();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_point_cloud_;
    ros::Publisher pub_grid_map_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 处理相关函数
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    SingleLineLaserData pointCloudToSingleLineLaser(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr detectObstacles(const SingleLineLaserData& single_line_data);
    GridMap generateGridMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_cloud, const std::string& frame_id);
    void publishGridMap(const GridMap& grid_map, const std::string& frame_id);
    bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud, const std::string& target_frame);
};

// 节点类的构造函数实现
ObstaclePerceptionNode::ObstaclePerceptionNode(ros::NodeHandle& nh) :
        nh_(nh),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>()),
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
    // 订阅点云话题（这里假设点云发布的话题名为 "/point_cloud"，可根据实际调整）
    sub_point_cloud_ = nh_.subscribe("/rslidar_points", 10, &ObstaclePerceptionNode::pointCloudCallback, this);
    // 初始化发布器，发布栅格地图话题（这里假设话题名为 "/grid_map"，可按需更改）
    pub_grid_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid_map", 10);
}

// 节点类的析构函数实现
ObstaclePerceptionNode::~ObstaclePerceptionNode() {}

// 回调函数实现（处理接收到的点云数据）
void ObstaclePerceptionNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud_ptr);

    SingleLineLaserData single_line_data = pointCloudToSingleLineLaser(cloud_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud = detectObstacles(single_line_data);

    if (transformPointCloud(obstacle_cloud, obstacle_cloud, "/vehicle_frame")) {
        GridMap grid_map = generateGridMap(obstacle_cloud, cloud_msg->header.frame_id);
        publishGridMap(grid_map, cloud_msg->header.frame_id);
    } else {
        ROS_ERROR("Failed to transform point cloud to vehicle frame.");
    }
}

// 点云转单线雷达数据函数实现
SingleLineLaserData ObstaclePerceptionNode::pointCloudToSingleLineLaser(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr) {
    SingleLineLaserData single_line_data;
    single_line_data.ranges.resize(cloud_ptr->points.size());
    single_line_data.intensities.resize(cloud_ptr->points.size());

    for (size_t i = 0; i < cloud_ptr->points.size(); ++i) {
        single_line_data.ranges[i] = std::sqrt(cloud_ptr->points[i].x * cloud_ptr->points[i].x +
                                               cloud_ptr->points[i].y * cloud_ptr->points[i].y +
                                               cloud_ptr->points[i].z * cloud_ptr->points[i].z);
        single_line_data.intensities[i] = cloud_ptr->points[i].intensity;
    }
    return single_line_data;
}

// 检测障碍物点云函数实现（简单示例，可根据实际优化算法）
pcl::PointCloud<pcl::PointXYZI>::Ptr ObstaclePerceptionNode::detectObstacles(const SingleLineLaserData& single_line_data) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    float obstacle_distance_threshold = 5.0;

    for (size_t i = 0; i < single_line_data.ranges.size(); ++i) {
        if (single_line_data.ranges[i] < obstacle_distance_threshold) {
            pcl::PointXYZI point;
            point.x = single_line_data.ranges[i] * std::cos(i * 0.01);
            point.y = single_line_data.ranges[i] * std::sin(i * 0.01);
            point.z = 0;
            point.intensity = single_line_data.intensities[i];
            obstacle_cloud->points.push_back(point);
        }
    }
    return obstacle_cloud;
}

// 生成栅格地图函数实现
GridMap ObstaclePerceptionNode::generateGridMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_cloud, const std::string& frame_id) {
    GridMap grid_map;
    grid_map.width = 100;
    grid_map.height = 100;
    grid_map.resolution = 0.1;
    grid_map.grid_data.resize(grid_map.width * grid_map.height, false);

    for (const auto& point : obstacle_cloud->points) {
        int grid_x = static_cast<int>(point.x / grid_map.resolution);
        int grid_y = static_cast<int>(point.y / grid_map.resolution);
        if (grid_x >= 0 && grid_x < grid_map.width && grid_y >= 0 && grid_y < grid_map.height) {
            grid_map.grid_data[grid_y * grid_map.width + grid_x] = true;
        }
    }
    return grid_map;
}

// 发布栅格地图函数实现
void ObstaclePerceptionNode::publishGridMap(const GridMap& grid_map, const std::string& frame_id) {
    nav_msgs::OccupancyGrid grid_map_msg;
    grid_map_msg.header.frame_id = frame_id;
    grid_map_msg.header.stamp = ros::Time::now();
    grid_map_msg.info.width = grid_map.width;
    grid_map_msg.info.height = grid_map.height;
    grid_map_msg.info.resolution = grid_map.resolution;
    grid_map_msg.data = std::vector<int8_t>(grid_map.grid_data.size(), 0);
    for (size_t i = 0; i < grid_map.grid_data.size(); ++i) {
        grid_map_msg.data[i] = grid_map.grid_data[i]? 100 : 0;
    }

    pub_grid_map_.publish(grid_map_msg);
}

// 坐标变换函数，将点云从源坐标系变换到目标坐标系（这里是转换到自车坐标系）
bool ObstaclePerceptionNode::transformPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                                                  pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud,
                                                  const std::string& target_frame) {
    try {
        geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(target_frame, input_cloud->header.frame_id, ros::Time(0), ros::Duration(3.0));
        Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);  // 将geometry_msgs::TransformStamped转换为Eigen::Affine3d

        output_cloud->points.clear();  // 清空输出点云
        for (const auto& point : input_cloud->points) {
            Eigen::Vector4d point_vec(point.x, point.y, point.z, 1.0);  // 将点坐标表示为齐次坐标形式
            Eigen::Vector4d transformed_point_vec = eigen_transform * point_vec;  // 利用Eigen进行坐标变换
            pcl::PointXYZI transformed_point;
            transformed_point.x = transformed_point_vec[0];
            transformed_point.y = transformed_point_vec[1];
            transformed_point.z = transformed_point_vec[2];
            transformed_point.intensity = point.intensity;
            output_cloud->points.push_back(transformed_point);
        }

        return true;
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_perception_node");
    ros::NodeHandle nh;
    ObstaclePerceptionNode perception_node(nh);
    ros::spin();
    return 0;
}