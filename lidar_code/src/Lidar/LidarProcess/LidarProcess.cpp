#include <Lidar/LidarProcess/LidarProcess.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


void LidarProcess(LiDAR& st_LiDAR, double timestamp)
{


    // 매 프레임 시작 시 이전 데이터를 초기화
    // 포인트클라우드(Ptr)들은 ->clear()를 사용
    st_LiDAR.pcl_RoiCloud->clear();
    st_LiDAR.pcl_VoxelCloud->clear();
    st_LiDAR.pcl_GroundCloud->clear();
    st_LiDAR.pcl_NonGroundCloud->clear();
    

    // std::vector들은 .clear()를 사용
    st_LiDAR.cluster_indices.clear();
    st_LiDAR.vec_Objects.clear();

    Roi(st_LiDAR);

    Voxel(st_LiDAR);

    Ransac(st_LiDAR);
    // EstimateGroundPatchwork(st_LiDAR);
    
    Clustering(st_LiDAR);

    FilterAndMergeClusters(st_LiDAR);

    L_ShapeFitting(st_LiDAR);

    ObjectTracking(st_LiDAR, timestamp);
}