# include "planning/localization/localization.hpp"
# include "planning/objective_function/objective_function.hpp"
# include <morai_msgs/EgoVehicleStatus.h>
# include <lidar_code/Track.h>

using namespace std;

extern morai_msgs::EgoVehicleStatus::ConstPtr vel_msg;

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
    
    // [수정] 절대 속도 (ENU) = 센서 프레임 변환 속도
    // ego 상대 속도가 아닌 절대 속도 사용 (절대 좌표에서의 장애물 속도)
    obstaclePose.obs_vel_e = obs_vel_sensor_e;
    obstaclePose.obs_vel_n = obs_vel_sensor_n;
    
    // [수정] 속도 크기: 절대 속도 기반
    double obs_abs_speed = std::sqrt(obs_vel_sensor_e * obs_vel_sensor_e + obs_vel_sensor_n * obs_vel_sensor_n) * 3.6; // km/h
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


bool loadPath() {
    // ros::spin() 이 돌면서 매번 path.txt 파일을 열 것을 대비해 if문으로 egoPath에 숫자double ax, double ay, double cx, double cy, double e, double n, double u열이 들어있으면 다시 열지 말것
    if (!egoPath_vec.empty()) {
        return true;
    }

    // ros가 find_path 패키지의 절대경로를 찾음
    string pkg_path = ros::package::getPath("contest");
    // 패키지 안에 파일이 들어있는 디렉토리를 언급해줌으로써 절대 경로 생성
    string file_path = pkg_path + "/src/path.txt";

    ifstream inputFile;
    inputFile.open(file_path);

    // file 열기 실패했을때의 if문 -> 디버깅 지점 확인 용도
    if (!inputFile.is_open()) {
        cout << "파일 열기에 실패 !" << endl;
        return false;
    }

    double e;
    double n;
    double u;
    while (inputFile >> e >> n >> u) {
        egoPath_vec.push_back(egoPath_struc{e, n, u});
    }

    if (egoPath_vec.empty()) {
        cout << "egoPath가 비어있습니다 !" << endl;
        return false;
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