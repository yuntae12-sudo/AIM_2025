#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Lidar/LidarProcess/LidarProcess.hpp>
#include <Global/Global.hpp>
#include <Visualizer/Visualizer.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <lidar_code/TrackArray.h>

// 전역 변수
LiDAR st_LiDARData;
ros::Publisher pub_box_visualizer;
ros::Publisher pub_cluster_visualizer;
ros::Publisher pub_roi;
ros::Publisher pub_voxel;
ros::Publisher pub_non_ground;
ros::Publisher pub_ground;
ros::Publisher pub_track_visualizer;
ros::Publisher pub_tracks_to_planning; //판제팀 전용 토픽

void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {

    pcl::fromROSMsg(*msg, *st_LiDARData.pcl_RawCloud);

    LidarProcess(st_LiDARData, msg->header.stamp.toSec());

    // msg->header.frame_id를 그대로 넘겨주면 라이다 좌표계에 맞춰서 그려짐
    PublishPointCloud(pub_roi, st_LiDARData.pcl_RoiCloud, msg->header.frame_id);
    
    // Voxel 결과 (점들이 듬성듬성해진 상태)
    PublishPointCloud(pub_voxel, st_LiDARData.pcl_VoxelCloud, msg->header.frame_id);

    // Ransac 결과
    PublishPointCloud(pub_ground, st_LiDARData.pcl_GroundCloud, msg->header.frame_id);
    
    PublishPointCloud(pub_non_ground, st_LiDARData.pcl_NonGroundCloud, msg->header.frame_id);

    // Clustering 결과
    PublishClusters(pub_cluster_visualizer, st_LiDARData, msg->header.frame_id);

    // Lshapefitting 결과
    PublishObjectBoundingBox(pub_box_visualizer, st_LiDARData.vec_Objects, "base_link");

    // 트래킹
    PublishTracks(pub_track_visualizer, st_LiDARData.vec_Tracks, "base_link");


    lidar_code::TrackArray msg_tracks;
    msg_tracks.header.stamp = ros::Time::now();
    msg_tracks.header.frame_id = "base_link";

    msg_tracks.tracks.reserve(st_LiDARData.vec_Tracks.size());

    for (const Track& t : st_LiDARData.vec_Tracks) {
        if (t.s32_LostFrames > 0) continue;

        lidar_code::Track track_data;


        // ---------------------------------------------------------
        // 데이터 채우기
        // ---------------------------------------------------------
        track_data.id = t.s32_ID;
        
        // Planning 노드가 경로를 예측할 수 있도록 위치는 '중심점'을 보냄
        track_data.x = t.f32_X_center;
        track_data.y = t.f32_Y_center;
        
        // 속도 및 가속도 (모서리의 변화율이 곧 물체의 변화율이므로 그대로 사용)
        track_data.vx = t.vec_X[2];
        track_data.vy = t.vec_X[3];
        track_data.ax = t.vec_X[4];
        track_data.ay = t.vec_X[5];

        // 크기 및 헤딩
        track_data.length  = t.f32_L;
        track_data.width   = t.f32_W;
        track_data.height  = t.f32_H;
        track_data.heading = t.f32_Heading;

        msg_tracks.tracks.push_back(track_data);
    }
        
    pub_tracks_to_planning.publish(msg_tracks);

    ROS_INFO("Objects Detected: %zu", st_LiDARData.vec_Tracks.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_tracker", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    pub_roi = nh.advertise<sensor_msgs::PointCloud2>("/lidar_roi", 1);
    pub_voxel = nh.advertise<sensor_msgs::PointCloud2>("/lidar_voxel", 1);

    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/lidar_ground", 1);
    pub_non_ground = nh.advertise<sensor_msgs::PointCloud2>("/lidar_non_ground", 1);

    pub_cluster_visualizer = nh.advertise<sensor_msgs::PointCloud2>("/lidar_clusters_visual", 1);
    
    // (MarkerArray 타입)
    pub_box_visualizer = nh.advertise<visualization_msgs::MarkerArray>("/lidar_objects_boxes", 1);

    pub_track_visualizer = nh.advertise<visualization_msgs::MarkerArray>("tracking_result", 1);

    pub_tracks_to_planning = nh.advertise<lidar_code::TrackArray>("target_tracks", 1);


    // 객체 없이 함수 직접 등록
    ros::Subscriber sub = nh.subscribe("/lidar3D", 1, LidarCallback);

    ros::spin();
    return 0;
}