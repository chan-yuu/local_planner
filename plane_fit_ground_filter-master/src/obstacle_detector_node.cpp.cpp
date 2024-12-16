#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <algorithm>
#include <tf2_ros/transform_listener.h>  // 使用tf2_ros
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/transforms.h>  // 包含pcl_ros的变换函数
#include <pcl/filters/voxel_grid.h>

// 定义用于表示单线雷达数据的结构体（这里简单示意，可以根据实际需求调整结构内容）
struct SingleLineLaserData {
    std::vector<float> ranges;
    std::vector<float> intensities;  // 改为float类型，与pcl::PointXYZI的强度类型匹配（如果使用该类型表示点云）
};

// 定义用于表示栅格地图的结构体（简单示意，可按需扩展完善）
struct GridMap {
    int width;
    int height;
    float resolution;
    std::vector<bool> grid_data;
};

// 新的节点类，用于实现后续感知算法
class ObstaclePerceptionNode {
public:
    ObstaclePerceptionNode(ros::NodeHandle& nh);
    ~ObstaclePerceptionNode();

private:
    // ROS相关对象
    ros::NodeHandle nh_;
    ros::Subscriber sub_point_cloud_;
    ros::Publisher pub_grid_map_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  // 使用tf2_ros::Buffer
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 处理相关函数
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    SingleLineLaserData pointCloudToSingleLineLaser(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr);  // 修改参数类型为pcl::PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr detectObstacles(const SingleLineLaserData& single_line_data);  // 修改返回类型及参数类型
    GridMap generateGridMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_cloud, const std::string& frame_id);  // 修改参数类型
    void publishGridMap(const GridMap& grid_map, const std::string& frame_id);
    bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud,
                             const std::string& target_frame);  // 修改参数类型
};

// 节点类的构造函数实现
ObstaclePerceptionNode::ObstaclePerceptionNode(ros::NodeHandle& nh) : nh_(nh), tf_buffer_(std::make_shared<tf2_ros::Buffer>()),
                                                                      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
    // 订阅点云话题（这里假设点云发布的话题名为 "/point_cloud"，可根据实际调整）
    sub_point_cloud_ = nh_.subscribe("/point_cloud", 10, &ObstaclePerceptionNode::pointCloudCallback, this);

    // 初始化发布器，发布栅格地图话题（这里假设话题名为 "/grid_map"，可按需更改）
    pub_grid_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid_map", 10);
}

// 节点类的析构函数实现
ObstaclePerceptionNode::~ObstaclePerceptionNode() {}

// 回调函数实现（处理接收到的点云数据）
void ObstaclePerceptionNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 将ROS消息转换为PCL点云数据类型
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud_ptr);

    // 步骤1：将点云处理成单线雷达数据
    SingleLineLaserData single_line_data = pointCloudToSingleLineLaser(cloud_ptr);

    // 步骤2：利用单线雷达数据检测障碍物点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud = detectObstacles(single_line_data);

    // 步骤3：将障碍物点云转换到自车坐标系下
    if (transformPointCloud(obstacle_cloud, obstacle_cloud, "vehicle_frame")) {
        // 步骤4：生成栅格数据
        GridMap grid_map = generateGridMap(obstacle_cloud, cloud_msg->header.frame_id);

        // 步骤5：发布栅格地图
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

    // 这里简单地提取每个点的距离信息作为单线雷达数据（可以根据实际需求，比如按照角度范围等更合理地提取）
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
    // 这里简单地设定一个距离阈值（例如，小于某个距离认为是障碍物，可根据实际情况调整）
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
    // 设置栅格地图的基本参数（示例值，需根据实际场景调整）
    grid_map.width = 100;
    grid_map.height = 100;
    grid_map.resolution = 0.1;

    grid_map.grid_data.resize(grid_map.width * grid_map.height, false);

    // 根据障碍物点云填充栅格数据（简单示例，可优化坐标转换和填充逻辑）
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
bool ObstaclePerceptionNode::transformPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud,
                                                  const std::string& target_frame) {
    try {
        geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(target_frame, input_cloud->header.frame_id, ros::Time(0), ros::Duration(3.0));

        Eigen::Affine3d eigen_transform;
        tf2::fromMsg(transform.transform, eigen_transform);

        pcl::transformPointCloud(*input_cloud, *output_cloud, eigen_transform);
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