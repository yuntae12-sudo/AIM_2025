#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Lidar/LidarProcess/LidarProcess.hpp>
#include <Global/Global.hpp>
#include <Visualizer/Visualizer.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <lidar_code/TrackArray.h>
#include <morai_msgs/EgoVehicleStatus.h> // MORAI 메시지 헤더 포함
#include <morai_msgs/GPSMessage.h>
#include <sensor_msgs/Imu.h>
#include <mutex>  

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

// 전역 변수: 실시간 차량 상태 저장용
double current_ego_x = 0.0;
double current_ego_y = 0.0;
double current_ego_heading = 0.0;
std::mutex ego_mutex;

// 판단 팀의 GPS 변환용 상수 및 구조체 (판단 팀 코드에서 가져오세요)
const double a = 6378137.0;
const double e = 0.006694379991;
const double pi = M_PI;
const double ref_lat = 37.238838359501933;
const double ref_lon = 126.772902206454901;

// 판단 팀의 GPS 변환 콜백
void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(ego_mutex);
    double rad_lat = msg->latitude * pi / 180;
    double rad_lon = msg->longitude * pi / 180;
    double k = a / sqrt(1 - e * pow(sin(rad_lat), 2));

    double ecef_x = k * cos(rad_lat) * cos(rad_lon);
    double ecef_y = k * cos(rad_lat) * sin(rad_lon);
    double ecef_z = k * (1 - e) * sin(rad_lat);

    double ref_rad_lat = ref_lat * pi / 180;
    double ref_rad_lon = ref_lon * pi / 180;
    double ref_k = a / sqrt(1 - e * pow(sin(ref_rad_lat), 2));

    double ref_ecef_x = ref_k * cos(ref_rad_lat) * cos(ref_rad_lon);
    double ref_ecef_y = ref_k * cos(ref_rad_lat) * sin(ref_rad_lon);
    double ref_ecef_z = ref_k * (1 - e) * sin(ref_rad_lat);

    // 인지 팀이 사용할 전역 변수에 저장 (East -> x, North -> y)
    current_ego_x = (-sin(rad_lon)*(ecef_x - ref_ecef_x) + cos(rad_lon)*(ecef_y - ref_ecef_y));
    current_ego_y = (-sin(rad_lat)*cos(rad_lon)*(ecef_x - ref_ecef_x) - sin(rad_lat)*sin(rad_lon)*(ecef_y - ref_ecef_y) + cos(rad_lat)*(ecef_z - ref_ecef_z));
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(ego_mutex);
    double imu_x = msg->orientation.x;
    double imu_y = msg->orientation.y;
    double imu_z = msg->orientation.z;
    double imu_w = msg->orientation.w;

    // 판단 팀 공식 그대로 이식 (Radian 단위)
    current_ego_heading = atan2(2 * (imu_w * imu_z + imu_x * imu_y), 1 - 2 * (pow(imu_y, 2) + pow(imu_z, 2)));
}

void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {

        double ego_x, ego_y, ego_heading;
    {
        std::lock_guard<std::mutex> lock(ego_mutex);
        ego_x = current_ego_x;
        ego_y = current_ego_y;
        ego_heading = current_ego_heading;
    }

    static ros::Time last_msg_stamp(0.0);          // 이전 프레임 시간 (Hz 계산용)
    ros::WallTime t_start = ros::WallTime::now();  // 실제 처리 시간 측정용


    pcl::fromROSMsg(*msg, *st_LiDARData.pcl_RawCloud);

    // LidarProcess(st_LiDARData, msg->header.stamp.toSec(), current_ego_x, current_ego_y, current_ego_heading);
    LidarProcess(st_LiDARData,
                 msg->header.stamp.toSec(),
                 ego_x, ego_y, ego_heading);   // ★ current_ego_* 말고 로컬값 사용

    const std::string lidar_frame = msg->header.frame_id;
    const ros::Time&   stamp      = msg->header.stamp;

    PublishPointCloud(pub_roi,        st_LiDARData.pcl_RoiCloud,       lidar_frame, stamp);
    PublishPointCloud(pub_voxel,      st_LiDARData.pcl_VoxelCloud,     lidar_frame, stamp);
    PublishPointCloud(pub_ground,     st_LiDARData.pcl_GroundCloud,    lidar_frame, stamp);
    PublishPointCloud(pub_non_ground, st_LiDARData.pcl_NonGroundCloud, lidar_frame, stamp);

    PublishClusters(pub_cluster_visualizer, st_LiDARData, lidar_frame, stamp);
    PublishObjectBoundingBox(pub_box_visualizer, st_LiDARData.vec_Objects, "base_link", stamp);
    PublishTracks(pub_track_visualizer,       st_LiDARData.vec_Tracks,     "base_link", stamp);



    lidar_code::TrackArray msg_tracks;
    // msg_tracks.header.stamp = ros::Time::now();
    msg_tracks.header.stamp    = stamp; 
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

    ros::WallDuration proc = ros::WallTime::now() - t_start;
    double proc_ms = proc.toSec() * 1000.0;   // [ms]
    double fps     = (proc_ms > 0.0) ? 1000.0 / proc_ms : 0.0;


    //  라이다 입력 Hz 계산
    if (!last_msg_stamp.isZero()) {
        double dt = (msg->header.stamp - last_msg_stamp).toSec();

        if (dt > 0.0) {
            double hz = 1.0 / dt;
            double util = (proc_ms / (dt * 1000.0)) * 100.0;

            ROS_INFO("[Pipeline] dt=%.3f s (%.1f Hz) | proc=%.2f ms | util=%.1f%% | FPS=%.1f",
                     dt, hz, proc_ms, util, fps);
        }
    }
    last_msg_stamp = msg->header.stamp;
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

    ros::Subscriber sub_gps = nh.subscribe("/gps", 1, gpsCallback);

    ros::Subscriber sub_imu = nh.subscribe("/imu", 1, imuCallback);
    // 객체 없이 함수 직접 등록
    ros::Subscriber sub = nh.subscribe("/lidar3D", 1, LidarCallback);

    ros::MultiThreadedSpinner spinner(3); // 최대 3개의 스레드로 콜백 처리
    spinner.spin();
    return 0;
}