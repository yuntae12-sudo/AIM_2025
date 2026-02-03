#ifndef LIDARPROCESS_HPP
#define LIDARPROCESS_HPP

#include <Global/Global.hpp>
#include <Lidar/RoiVoxel/RoiVoxel.hpp>
#include <Lidar/Ransac/Ransac.hpp>
#include <Lidar/Clustering/Clustering.hpp>
#include <Lidar/Lshapefitting/LshapeFitting.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <Lidar/ObjectTracking/ObjectTracking.hpp>
#include <Lidar/FilterAndMergeClusters/FilterAndMergeClusters.hpp>
#include <Lidar/Boundary/Boundary.hpp>

void LidarProcess(LiDAR& st_LiDAR, double timestamp, double ego_x, double ego_y, double ego_heading);
#endif