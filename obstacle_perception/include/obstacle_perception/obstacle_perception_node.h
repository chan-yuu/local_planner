/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-12-17 00:49:10
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-12-17 00:50:47
 * @FilePath: /undefined/home/cyun/my_project/local_map_ws/src/obstacle_perception/include/obstacle_perception/obstacle_perception_node.h
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#ifndef OBSTACLE_PERCEPTION_NODE_H
#define OBSTACLE_PERCEPTION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

struct SingleLineLaserData {
    std::vector<float> ranges;
    std::vector<float> intensities;
};

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

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    SingleLineLaserData pointCloudToSingleLineLaser(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr detectObstacles(const SingleLineLaserData& single_line_data);
    GridMap generateGridMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_cloud, const std::string& frame_id);
    void publishGridMap(const GridMap& grid_map, const std::string& frame_id);
    bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud, const std::string& target_frame);
};

#endif  // OBSTACLE_PERCEPTION_NODE_H