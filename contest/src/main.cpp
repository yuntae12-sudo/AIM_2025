# include <ros/ros.h>
# include <stdio.h>
# include <iostream>
// 수정 부분 
# include <map>
# include <limits>

# include "planning/localization/localization.hpp"
# include "planning/localization/obstacle_tracker.hpp"
# include "planning/objective_function/objective_function.hpp"
# include "visualization/visualization.hpp"
# include "planning/gps_jamming/jamming.hpp"

# include <morai_msgs/GPSMessage.h>
# include <morai_msgs/EgoVehicleStatus.h>
# include <morai_msgs/CtrlCmd.h>
# include <sensor_msgs/Imu.h>
# include <visualization_msgs/Marker.h>
# include <visualization_msgs/MarkerArray.h>
// #include <std_msgs/Float64MultiArray.h>

// 수정 부분
# include <lidar_code/TrackArray.h>
# include <lidar_code/Track.h>

using namespace std;

// [추가] ObstacleTracker 전역 객체
ObstacleTracker g_obstacle_tracker;

ros::Publisher control_pub;
ros::Publisher waypoints_pub;
ros::Publisher candidate_paths_pub;
ros::Publisher best_path_pub;
ros::Publisher boundary_pub;

ros::Subscriber sub_gps;
ros::Subscriber sub_imu;
ros::Subscriber sub_vel;
ros::Subscriber sub_objects;
ros::Subscriber sub_obj;
ros::Subscriber sub_cam;


double L_d;  // look-ahead distance
double k;    // look-forward gain
double gain_ld;  // look-ahead distance
double gain_k;    // look-forward gain

RobotConstants roboconsts;
egoPose_struc egoPose;
egoVelocity_struc egoVelocity;
Obstacle_struct obstacle;

int last_closest_idx = 0;
string active_path_param_name;

morai_msgs::EgoVehicleStatus::ConstPtr vel_msg;
morai_msgs::GPSMessage::ConstPtr gps_msg;
contest::LaneInfo::ConstPtr lane_msg;

map<int, Obstacle_struct> obstacles_map;

void gpsCallback (const morai_msgs::GPSMessage::ConstPtr& msg) {
    gps_msg = msg;
    gps2Enu(msg, egoPose);
}

void imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg) {
    yawTf(imu_msg, egoPose);
}

void obsCallback(const lidar_code::TrackArray::ConstPtr& track_msg) {
    // 1. 센서에서 들어온 "날것의" 장애물들을 먼저 ENU로 변환
    vector<Obstacle_struct> raw_obstacles;
    
    for (const auto& track_data : track_msg->tracks) {
        Obstacle_struct obs;
        
        // 좌표 및 속도 계산 (센서 프레임 → ENU 변환)
        obsCordinate(track_data, obs, egoPose, vel_msg);
        obsVelocity(track_data, obs, egoPose, egoVelocity, vel_msg);
        
        // 장애물 기본 정보 업데이트
        obs.id = track_data.id;
        obs.length = track_data.length;
        obs.width = track_data.width;
        obs.heading = track_data.heading;
        
        // 반지름 설정: 대각선 길이의 절반 (안전 마진)
        obs.radius = sqrt(pow(track_data.length, 2) + pow(track_data.width, 2)) / 2.0;
        if (obs.radius < 0.1) obs.radius = 0.1;
        
        raw_obstacles.push_back(obs);
    }
    
    // [추가] 2. 칼만 필터 기반 추적기를 통해 안정화
    // raw_obstacles에서 튀는 위치들을 필터링해서 stable_obstacles 생성
    vector<Obstacle_struct> stable_obstacles = g_obstacle_tracker.updateObstacles(raw_obstacles);
    
    // [수정] 3. 안정화된 위치는 사용하되, 원본 센서 속도는 보존
    obstacles_map.clear();
    
    // raw_obstacles를 ID로 인덱싱 (빠른 조회)
    map<int, Obstacle_struct> raw_obs_map;
    for (const auto& obs : raw_obstacles) {
        raw_obs_map[obs.id] = obs;
    }
    
    
    // [수정] 중복 제거: 각 ID당 하나씩만 저장
    map<int, Obstacle_struct> unique_stable_obs;
    for (auto obs : stable_obstacles) {
        if (unique_stable_obs.find(obs.id) == unique_stable_obs.end()) {
            unique_stable_obs[obs.id] = obs;
        }
    }
    
    for (auto& pair : unique_stable_obs) {
        auto& obs = pair.second;
        
        // 원본 센서 데이터에서 해당 ID의 속도를 찾아 사용
        if (raw_obs_map.find(obs.id) != raw_obs_map.end()) {
            
            obs.obs_vel_e = raw_obs_map[obs.id].obs_vel_e;
            obs.obs_vel_n = raw_obs_map[obs.id].obs_vel_n;
            obs.obs_speed = raw_obs_map[obs.id].obs_speed;
        } else {
            // raw data가 없으면 속도 0으로 설정 (old tracked data는 사용 안 함)
            obs.obs_vel_e = 0.0;
            obs.obs_vel_n = 0.0;
            obs.obs_speed = 0.0;
        }
        obstacles_map[obs.id] = obs;
    }
    
    // [디버그 출력] 원본 vs 안정화된 개수 비교 (10프레임마다)
    static int debug_counter = 0;
    if (++debug_counter % 10 == 0) {
        ROS_DEBUG("Obstacles: Raw[%zu] → Tracked[%zu]", raw_obstacles.size(), stable_obstacles.size());
    }
}

void laneCallback (const contest::LaneInfo::ConstPtr& msg) {
    lane_msg = msg;
}

void mainCallback (const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {

    vel_msg = msg;

    if(!gps_msg) return;
    if (egoPath_vec.empty()) {
        ROS_WARN_THROTTLE(1.0, "Active path is empty. Skip control cycle.");
        return;
    }
    // if(!lane_msg) return;

    int requested_path_id = getActivePathId();
    if (!active_path_param_name.empty() &&
        ros::param::getCached(active_path_param_name, requested_path_id) &&
        requested_path_id != getActivePathId()) {
        if (setActivePath(requested_path_id)) {
            last_closest_idx = -1; // path 변경 직후에는 전체 탐색으로 인덱스 재동기화
            ROS_INFO("Runtime path switch -> id=%d", requested_path_id);
        } else {
            ROS_WARN_THROTTLE(1.0, "Requested active_path_id=%d is not available", requested_path_id);
        }
    }

    bodyframe2Enu(egoPose, egoVelocity, msg);

    double steering_angle  = 0.0;

    if(Is_Jamming(gps_msg)) {
        steering_angle = GpsJamming(gps_msg, lane_msg);

        cout << "steer_angle: " << steering_angle << endl;
    }
    else {

        vector<Obstacle_struct> obstacles;
        for (const auto& pair : obstacles_map) {
            obstacles.push_back(pair.second);
        }
        const Obstacle_struct* closest_obstacle = nullptr;
        double closest_dist = std::numeric_limits<double>::max();
        for (const auto& obs : obstacles) {
            const double dist = hypot(obs.e - egoPose.current_e, obs.n - egoPose.current_n);
            if (dist < closest_dist) {
                closest_dist = dist;
                closest_obstacle = &obs;
            }
        }

        // Mode selection and sampling/weight preparation
        mode_struct mode;
        sampling_struct sampling;
        Weight_struct weight;
        modeCheck(mode, obstacles, egoPose);

        // only linear or coner modes supported here
        linearMode(mode, sampling, weight);
        conerMode(mode, sampling, weight);
        staticObstacleMode(mode, sampling, weight);
        if (closest_obstacle != nullptr) {
            dynamicObstacleMode(mode, sampling, weight, *closest_obstacle);
        }
        const int path_id_before_manual_mode = getActivePathId();
        manualMode(mode, sampling, weight);
        const int path_id_after_manual_mode = getActivePathId();
        if (path_id_before_manual_mode != path_id_after_manual_mode) {
            last_closest_idx = -1; // manual 종료/진입 시 경로가 바뀌면 인덱스 히스토리 리셋
        }
        if (!active_path_param_name.empty()) {
            int current_param_path_id = path_id_after_manual_mode;
            if (ros::param::getCached(active_path_param_name, current_param_path_id) &&
                current_param_path_id != path_id_after_manual_mode) {
                ros::param::set(active_path_param_name, path_id_after_manual_mode);
            }
        }

        int current_path_idx = getCurrentIndex(egoPath_vec, egoPose, last_closest_idx);
        last_closest_idx = current_path_idx; // 다음을 위해 저장

        generateCandidates(Candidate_vec, egoPath_vec, egoPose, msg, roboconsts, obstacles, in_boundary_vec, out_boundary_vec, sampling, weight);
        evaluateCandidates(Candidate_vec, obstacles, egoPath_vec, egoPose, egoVelocity, msg, sampling, weight, current_path_idx);

        Candidate_struct best_candidate = selectBestCandidate(Candidate_vec);

        // 만약 유효한 경로가 없으면 비상 정지 (selectBestCandidate 내부나 여기서 처리)
        if (best_candidate.total_score <= -500) { // 점수가 너무 낮으면
            ROS_WARN("No valid path! Emergency Brake.");
            best_candidate.steer_angle = 0;
            // 속도도 0으로 줄이는 로직 필요
        }

        
        // 1. 핵심 인덱스 추출
        int closest_idx = findClosestPoint(egoPath_vec, egoPose);
        int target_idx = findWaypoint(egoPath_vec, egoPose, L_d);

        // 2. 오차값 계산 (모니터링용)
        double dist_err = getDistanceErr(egoPath_vec, egoPose);
        double yaw_err = getYawErr(egoPath_vec, egoPose, L_d);
        steering_angle = best_candidate.steer_angle;

        // [디버그 출력] 한 화면에 다 보이게 출력 (ROS_INFO_THROTTLE로 0.1초마다 출력 추천)
        // printf("\n[DEBUG] Pos: (%.2f, %.2f) | Yaw: %.2f deg\n", egoPose.current_e, egoPose.current_n, egoPose.current_yaw * 180/M_PI);
        // if (!egoPath_vec.empty()) {
        //     printf("        Idx: Close[%d], Target[%d] | Path_Pos: (%.2f, %.2f)\n", 
        //         closest_idx, target_idx, egoPath_vec[closest_idx].e, egoPath_vec[closest_idx].n);
        // } else {
        //     printf("        Idx: Close[%d], Target[%d] | Path empty\n", closest_idx, target_idx);
        // }
        // printf("        Err: Dist[%.2f m], Yaw[%.2f deg] | Steer: %.2f deg\n", 
        //         dist_err, yaw_err * 180/M_PI, steering_angle * 180/M_PI);
        // // [추가된 장애물 속도 출력 로직]
        // if (!obstacles.empty()) {
        //     printf(" [OBSTACLE SPEED INFO]\n");
        //     int obs_count = 0;
        //     for (const auto& obs : obstacles) {
        //         double speed = hypot(obs.obs_vel_e, obs.obs_vel_n);
        //         // 상위 3개 장애물만 출력 (터미널 스크롤 방지)
        //         printf("        ID(Map): [%d] | Vel_E: %6.2f, Vel_N: %6.2f | Speed: %6.2f m/s | radius: %6.2f m\n", 
        //                 obs_count, obs.obs_vel_e, obs.obs_vel_n, speed, obs.radius);
        //         printf("        obs_e: %6.2f, obs_n: %6.2f\n", obs.e, obs.n);
        //         if (++obs_count >= 3) break;
        //     }
        // } else {
        //     printf(" [OBSTACLE SPEED INFO] No obstacles detected.\n");
        // }

        string current_mode = "UNKNOWN";
        if (mode.manual_mode) current_mode = "MANUAL";
        else if (mode.dynamic_obstacle_mode) current_mode = "DYNAMIC_OBS";
        else if (mode.static_obstacle_mode) current_mode = "STATIC_OBS";
        else if (mode.coner_mode) current_mode = "CORNER";
        else if (mode.linear_mode) current_mode = "LINEAR";
        const int current_path_id = getActivePathId();
        string current_path_name = "path_unknown";
        if (current_path_id == 1) current_path_name = "path_01";
        else if (current_path_id == 2) current_path_name = "path_02";

        cout << fixed;
        cout.precision(4);
        cout << "============================================" << endl;
        cout << "[Best Candidate Score Detail]" << endl;
        cout << " Total Score   : " << best_candidate.total_score << endl;
        cout << " 1. Obs Term   : " << best_candidate.score_dist_obs << endl;
        cout << " 2. Vel Term   : " << best_candidate.score_vel << endl;
        cout << " 3. Head Term  : " << best_candidate.score_heading << endl;
        cout << " 4. Path Term  : " << best_candidate.score_dist_path << endl;
        cout << "--------------------------------------------" << endl;
        cout << " Selected V    : " << best_candidate.v << " m/s" << endl;
        cout << " Selected Steer: " << best_candidate.steer_angle << " rad" << endl;
        cout << "============================================" << endl;
        cout << "Current Mode: " << current_mode << endl;
        cout << "Active Path: " << current_path_name << " (id=" << current_path_id << ")" << endl;
        cout << "Current Index: " << closest_idx << " | Target Index: " << target_idx << endl;
        
        // [수정] obstacles 확인 후 접근
        if (closest_obstacle != nullptr) {
            // LiDAR 출력은 상대속도(base_link 기준)라 정지 장애물도 ego 속도와 유사하게 보일 수 있음
            const double rel_kmh = hypot(closest_obstacle->obs_vel_e, closest_obstacle->obs_vel_n) * 3.6;
            const double abs_vel_e = closest_obstacle->obs_vel_e + egoVelocity.ego_vel_e;
            const double abs_vel_n = closest_obstacle->obs_vel_n + egoVelocity.ego_vel_n;
            const double abs_kmh = hypot(abs_vel_e, abs_vel_n) * 3.6;
            cout << "Obstacle Speed (rel/abs): " << rel_kmh << " / " << abs_kmh << " km/h" << endl;
        } else {
            cout << "Obstacle Speed: No obstacles detected" << endl;
        }
        
        // 시각화 호출
        publishWaypoints(egoPath_vec);
        publishCandidates(Candidate_vec, candidate_paths_pub); 
        publishBestPath(best_candidate, best_path_pub);
        publishBoundaries(in_boundary_vec, out_boundary_vec);
    }

    velocityControl(msg, egoPose, egoVelocity);

    double accel_input = egoVelocity.accel_input;
    double brake_input = egoVelocity.brake_input;

    morai_msgs::CtrlCmd cmd_msg;
    cmd_msg.longlCmdType = 1;
    cmd_msg.accel = accel_input;
    cmd_msg.brake = brake_input;    
    cmd_msg.steering = steering_angle;

    cout << "Accel: " << accel_input << ", Brake: " << brake_input << ", Steering Angle: " << steering_angle << endl;

    control_pub.publish(cmd_msg);
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // read parameters with sensible defaults
    private_nh.param("gain_ld", gain_ld, 0.6);
    private_nh.param("gain_k", gain_k, 0.6);
    private_nh.param("look_ahead", L_d, 1.0);
    private_nh.param("look_k", k, 0.0);
    private_nh.param("active_path_id", active_path_id, 1);
    active_path_param_name = ros::this_node::getName() + "/active_path_id";

    if (!loadPathLibrary()) {
        ROS_ERROR("Failed to load path library. Shutting down.");
        return 1; // 오류 코드로 종료
    }
    if (!setActivePath(active_path_id)) {
        ROS_WARN("Requested active_path_id=%d not found. Keep current active path id=%d",
                 active_path_id, getActivePathId());
    }
    ROS_INFO("Path library ready. Active path id=%d, points=%lu",
             getActivePathId(), egoPath_vec.size());

    if (!loadBoundaries()) {
        ROS_ERROR("Failed to load path file. Shutting down.");
        return 1; // 오류 코드로 종료
    }
    // safe print
    ROS_INFO("Path loaded successfully with %lu points.", in_boundary_vec.size());
    ROS_INFO("Path loaded successfully with %lu points.", out_boundary_vec.size());

    control_pub = nh.advertise <morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
    waypoints_pub = nh.advertise<visualization_msgs::Marker>("viz_waypoints", 1);
    candidate_paths_pub = nh.advertise<visualization_msgs::MarkerArray>("viz_candidates", 1);
    best_path_pub = nh.advertise<visualization_msgs::Marker>("viz_best_candidate", 1); 
    boundary_pub = nh.advertise<visualization_msgs::Marker>("viz_boundaries", 1);

    sub_obj = nh.subscribe("/target_tracks", 1, obsCallback);
    sub_gps = nh.subscribe("/gps", 1, gpsCallback);
    sub_imu = nh.subscribe("/imu", 1, imuCallback);
    sub_vel = nh.subscribe("/Ego_topic", 1, mainCallback);
    sub_cam = nh.subscribe("/lane/path", 1, laneCallback);
    

    ros::spin();

    return 0;
}
