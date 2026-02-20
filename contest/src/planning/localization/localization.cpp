# include "planning/localization/localization.hpp"
# include "planning/objective_function/objective_function.hpp"
# include <morai_msgs/EgoVehicleStatus.h>
# include <lidar_code/Track.h>

using namespace std;

extern morai_msgs::EgoVehicleStatus::ConstPtr vel_msg;

vector<egoPath_struc> egoPath_vec;
vector<egoPath2_struc> egoPath2_vec;
map<int, vector<egoPath_struc>> path_library;
int active_path_id = 1;
vector<egoPath_struc> in_boundary_vec;
vector<egoPath_struc> out_boundary_vec;

double GetYawRate(const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg) {
    if(!vel_msg) return 0.0;

    double wheel_base = 3.0;
    double vel = hypot(pow(vel_msg->velocity.x, 2), pow(vel_msg->velocity.y, 2));
    double angle = vel_msg->wheel_angle;
    double steer_angle = angle * (pi / 180.0);
    return (vel / wheel_base) * tan(steer_angle);
}

void gps2Enu (const morai_msgs::GPSMessage::ConstPtr& gps_msg, egoPose_struc& egoPose) {
    // GPSMessage에서 받아올 WGS(GPS)값과 그 값에 대한 변수 설정
    double wgs_lat = gps_msg->latitude;
    double wgs_lon = gps_msg->longitude;
    double wgs_alt = gps_msg->altitude;
    // WGS값 radian으로 변경(계산 목적)
    double rad_lat = wgs_lat * pi / 180;
    double rad_lon = wgs_lon * pi / 180;
    double rad_alt = wgs_alt * pi / 180;
    double h = wgs_alt;
    double k = a / sqrt(1-e*pow(sin(rad_lat), 2));

    // WGS->ECEF
    double ecef_x = (k + h) * cos(rad_lat) * cos(rad_lon);
    double ecef_y = (k + h) * cos(rad_lat) * sin(rad_lon);
    double ecef_z = (k * (1-e) + h) * sin(rad_lat);

    double ref_rad_lat = ref_lat * pi / 180;
    double ref_rad_lon = ref_lon * pi / 180;
    double ref_rad_alt = ref_alt * pi / 180;

    double ref_h = ref_alt;
    double ref_k = a / sqrt(1 - e * pow(sin(ref_rad_lat), 2));
    double ref_ecef_x = (ref_k + ref_h) * cos(ref_rad_lat) * cos(ref_rad_lon);
    double ref_ecef_y = (ref_k + ref_h) * cos(ref_rad_lat) * sin(ref_rad_lon);
    double ref_ecef_z = (ref_k * (1 - e) + ref_h) * sin(ref_rad_lat);


    // ecef 좌표 enu 좌표로 변환
    // ref_rad_lon, ref_rad_lat (기준점 변수)를 사용해야 함
    egoPose.current_e = (-sin(ref_rad_lon) * (ecef_x - ref_ecef_x) + cos(ref_rad_lon) * (ecef_y - ref_ecef_y));

    egoPose.current_n = (-sin(ref_rad_lat) * cos(ref_rad_lon) * (ecef_x - ref_ecef_x) 
                         - sin(ref_rad_lat) * sin(ref_rad_lon) * (ecef_y - ref_ecef_y) 
                         + cos(ref_rad_lat) * (ecef_z - ref_ecef_z));
    double current_u = (cos(rad_lat)*cos(rad_lon)*(ecef_x - ref_ecef_x) + cos(rad_lat)*sin(rad_lon)*(ecef_y-ref_ecef_y) + sin(rad_lat)*(ecef_z-ref_ecef_z));
}

// 좌표계 변환 수정
void bodyframe2Enu (const egoPose_struc& egoPose, egoVelocity_struc& egoVelocity, const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg) {
    double v_x = vel_msg->velocity.x; // 차량 앞뒤 속도
    double v_y = vel_msg->velocity.y; // 차량 좌우 속도
    // double heading = vel_msg->heading * (pi / 180.0);

    // ENU 좌표계로 변환
    egoVelocity.ego_vel_e = v_x * cos(egoPose.current_yaw) - v_y * sin(egoPose.current_yaw);
    egoVelocity.ego_vel_n = v_x * sin(egoPose.current_yaw) + v_y * cos(egoPose.current_yaw);
}

void obsCordinate (const lidar_code::Track& track_data, Obstacle_struct& obstaclePose, const egoPose_struc& egoPose, const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg) {
    // track_msg에서 장애물 좌표 받아오기
    double obs_x = track_data.x;
    double obs_y = track_data.y;

    // [수정] 라이다 센서 프레임 좌표를 ENU 좌표로 변환
    obstaclePose.e = egoPose.current_e + (obs_x * cos(egoPose.current_yaw) - obs_y * sin(egoPose.current_yaw));
    obstaclePose.n = egoPose.current_n + (obs_x * sin(egoPose.current_yaw) + obs_y * cos(egoPose.current_yaw)) - 3.5;
    
}

void obsVelocity (const lidar_code::Track& track_data, Obstacle_struct& obstaclePose, const egoPose_struc& egoPose, const egoVelocity_struc& egoVelocity, const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg) {

    if(!vel_msg) return;
    
    // track_msg에서 장애물 속도 받아오기 (센서 프레임)
    double obs_vx = track_data.vx;
    double obs_vy = track_data.vy;
    
    // [수정] 라이다 센서 프레임 속도 → ENU 좌표계 속도 변환
    // 각속도 오류 제거: GetYawRate를 속도에서 빼지 말고, 속도 벡터만 회전 변환
    double obs_vel_sensor_e = obs_vx * cos(egoPose.current_yaw) - obs_vy * sin(egoPose.current_yaw);
    double obs_vel_sensor_n = obs_vx * sin(egoPose.current_yaw) + obs_vy * cos(egoPose.current_yaw);
    
    // [추가] 센서 속도가 0에 가까우면 위치 변화로 속도 추정 (더 정확한 추정)
    const double SENSOR_VEL_THRESHOLD = 0.1;  // m/s
    double sensor_speed_mag = hypot(obs_vel_sensor_e, obs_vel_sensor_n);
    
    if (sensor_speed_mag < SENSOR_VEL_THRESHOLD) {
        // 이전 위치가 저장되어 있으면 (초기값이 아니면)
        if (obstaclePose.prev_e != 0.0 || obstaclePose.prev_n != 0.0) {
            // 위치 변화로 속도 추정 (dt = 0.1초 가정, LiDAR 주파수 ~10Hz)
            double dt = 0.1;
            obstaclePose.obs_vel_e = (obstaclePose.e - obstaclePose.prev_e) / dt;
            obstaclePose.obs_vel_n = (obstaclePose.n - obstaclePose.prev_n) / dt;
        } else {
            obstaclePose.obs_vel_e = obs_vel_sensor_e;
            obstaclePose.obs_vel_n = obs_vel_sensor_n;
        }
    } else {
        // 센서 속도가 충분하면 그대로 사용
        obstaclePose.obs_vel_e = obs_vel_sensor_e;
        obstaclePose.obs_vel_n = obs_vel_sensor_n;
    }
    
    // [추가] 이전 위치 저장 (다음 프레임에서 사용)
    obstaclePose.prev_e = obstaclePose.e;
    obstaclePose.prev_n = obstaclePose.n;
    
    // [수정] 속도 크기: 절대 속도 기반
    double obs_abs_speed = std::sqrt(obstaclePose.obs_vel_e * obstaclePose.obs_vel_e + obstaclePose.obs_vel_n * obstaclePose.obs_vel_n) * 3.6; // km/h
    double ego_speed = vel_msg->velocity.x * 3.6;  // km/h
    
    // 상대 속도 (차량 기준)
    obstaclePose.obs_speed = obs_abs_speed - ego_speed;
}

bool is_obs_static(const Obstacle_struct& obs) {  // obs_speed가 speed_threshold보다 작으면 정지한 것으로 간주
    double speed_threshold = 10.0 * 3.6; // km/h
    return obs.obs_speed < speed_threshold;
}

void yawTf (const sensor_msgs::Imu::ConstPtr& imu_msg, egoPose_struc& egoPose) {
    double imu_x = imu_msg->orientation.x;
    double imu_y = imu_msg->orientation.y;
    double imu_z = imu_msg->orientation.z;
    double imu_w = imu_msg->orientation.w;
    // imu 쿼터니안 -> 오일러 변환 (yaw 각 범위 -180 < yaw < 180)
    egoPose.current_yaw = atan2(2 * (imu_w * imu_z + imu_x * imu_y), 1-2 * (pow(imu_y, 2) + pow(imu_z, 2)));
}


namespace {
bool loadPathFile(const string& file_name, vector<egoPath_struc>& out_path) {
    out_path.clear();

    const string pkg_path = ros::package::getPath("contest");
    const string file_path = pkg_path + "/src/" + file_name;
    ifstream input_file(file_path);

    if (!input_file.is_open()) {
        ROS_WARN("Failed to open path file: %s", file_path.c_str());
        return false;
    }

    double e = 0.0;
    double n = 0.0;
    double u = 0.0;
    while (input_file >> e >> n >> u) {
        out_path.push_back(egoPath_struc{e, n, u});
    }

    if (out_path.empty()) {
        ROS_WARN("Path file is empty: %s", file_path.c_str());
        return false;
    }
    return true;
}

void syncLegacyPath2Vector(const vector<egoPath_struc>& src_path) {
    egoPath2_vec.clear();
    egoPath2_vec.reserve(src_path.size());
    for (const auto& pt : src_path) {
        egoPath2_vec.push_back(egoPath2_struc{pt.e, pt.n, pt.u});
    }
}
} // namespace

bool loadPathById(int path_id, const std::string& file_name) {
    if (path_id <= 0) {
        ROS_ERROR("Invalid path_id: %d", path_id);
        return false;
    }

    const auto existing_it = path_library.find(path_id);
    if (existing_it != path_library.end() && !existing_it->second.empty()) {
        return true;
    }

    vector<egoPath_struc> loaded_path;
    if (!loadPathFile(file_name, loaded_path)) {
        return false;
    }

    path_library[path_id] = loaded_path;
    if (path_id == 2) {
        syncLegacyPath2Vector(loaded_path);
    }
    ROS_INFO("Loaded path id=%d from %s (%zu points)",
             path_id, file_name.c_str(), loaded_path.size());
    return true;
}

bool loadPathLibrary() {
    const bool path1_loaded = loadPathById(1, "path_01.txt") || loadPathById(1, "path.txt");
    const bool path2_loaded = loadPathById(2, "path_02.txt") || loadPathById(2, "path2.txt");

    if (!path1_loaded && !path2_loaded) {
        ROS_ERROR("No path files were loaded. Checked path_01/path_02 and path/path2.");
        return false;
    }

    if (path_library.find(active_path_id) == path_library.end()) {
        active_path_id = path1_loaded ? 1 : 2;
    }
    return setActivePath(active_path_id);
}

bool setActivePath(int path_id) {
    const auto it = path_library.find(path_id);
    if (it == path_library.end() || it->second.empty()) {
        ROS_ERROR("Cannot activate path id=%d (not loaded or empty)", path_id);
        return false;
    }

    egoPath_vec = it->second;
    active_path_id = path_id;
    ROS_INFO("Active path switched to id=%d (%zu points)", active_path_id, egoPath_vec.size());
    return true;
}

int getActivePathId() {
    return active_path_id;
}

bool loadPath() {
    const bool loaded = loadPathById(1, "path_01.txt") || loadPathById(1, "path.txt");
    if (!loaded) {
        return false;
    }
    return setActivePath(1);
}

bool loadPath2() {
    const bool loaded = loadPathById(2, "path_02.txt") || loadPathById(2, "path2.txt");
    if (!loaded) {
        return false;
    }
    const auto it = path_library.find(2);
    if (it != path_library.end()) {
        syncLegacyPath2Vector(it->second);
    }
    return true;
}

// [추가] 경계선 파일 로드 함수
bool loadBoundaries() {
    if (!in_boundary_vec.empty() && !out_boundary_vec.empty()) return true;

    string pkg_path = ros::package::getPath("contest");
    
    // 파일명은 실제 환경에 맞춰 수정해주세요 (예: in_boundary.txt)
    string in_file_path = pkg_path + "/src/in_boundary.txt"; 
    string out_file_path = pkg_path + "/src/out_boundary.txt";

    ifstream in_file(in_file_path);
    ifstream out_file(out_file_path);

    if (!in_file.is_open() || !out_file.is_open()) {
        cout << "경계선 파일 열기 실패!" << endl;
        return false;
    }

    double e, n, u;
    // In Boundary 로드
    while (in_file >> e >> n >> u) {
        in_boundary_vec.push_back(egoPath_struc{e, n, u});
    }
    // Out Boundary 로드
    while (out_file >> e >> n >> u) {
        out_boundary_vec.push_back(egoPath_struc{e, n, u});
    }

    cout << "Boundaries loaded. In: " << in_boundary_vec.size() 
         << ", Out: " << out_boundary_vec.size() << endl;

    return true;
}
