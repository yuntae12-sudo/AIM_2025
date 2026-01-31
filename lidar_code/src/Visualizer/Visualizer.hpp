#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Global/Global.hpp>

void PublishObjectBoundingBox(ros::Publisher& pub, const std::vector<Lbox>& vec_Objects, const std::string& frame_id = "lidar_link");
void PublishClusters(ros::Publisher& pub, const LiDAR& st_LiDAR, const std::string& frame_id = "lidar_link");
void PublishPointCloud(ros::Publisher& pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& frame_id = "lidar_link");
void PublishTracks(ros::Publisher& pub, const std::vector<Track>& vec_Tracks, const std::string& frame_id = "lidar_link");
#endif