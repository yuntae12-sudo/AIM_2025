# include <ros/ros.h>
# include <stdio.h>
# include <iostream>
// 수정 부분 
# include <map>

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
ros::Publisher ego_obb_pub;\
ros::Publisher obs_obb_pub;

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
vector<egoPath_struc> egoPath_vec;
vector<egoPath_struc> in_boundary_vec;
vector<egoPath_struc> out_boundary_vec;

int last_closest_idx = 0;

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
    
    // 3. 안정화된 장애물들을 전역 맵에 저장
    obstacles_map.clear();
    for (const auto& obs : stable_obstacles) {
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
    // if(!lane_msg) return;
    
    OBB ego_obb = GetEgoOBB(gps_msg, egoPose, roboconsts);

    bodyframe2Enu(egoPose, egoVelocity, msg);
    int current_path_idx = getCurrentIndex(egoPath_vec, egoPose, last_closest_idx);
    
    last_closest_idx = current_path_idx; // 다음을 위해 저장

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

        // Mode selection and sampling/weight preparation
        mode_struct mode;
        sampling_struct sampling;
        Weight_struct weight;
        modeCheck(mode, obstacles, egoPose);
        // only linear or coner modes supported here
        linearMode(mode, sampling, weight);
        conerMode(mode, sampling, weight);
        staticObstacleMode(mode, sampling, weight);
        dynamicObstacleMode(mode, sampling, weight, obstacles[0]); // Assuming first obstacle is the dynamic one

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
            // 시각화 호출
        publishWaypoints(egoPath_vec);
        publishCandidates(Candidate_vec, candidate_paths_pub); 
        publishBestPath(best_candidate, best_path_pub);
        publishBoundaries(in_boundary_vec, out_boundary_vec);
        publishEgoOBB(ego_obb);
        publishObstacleOBBs(obstacles);
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

    if (!loadPath()) {
        ROS_ERROR("Failed to load path file. Shutting down.");
        return 1; // 오류 코드로 종료
    }
    // safe print
    ROS_INFO("Path loaded successfully with %lu points.", egoPath_vec.size());

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
    ego_obb_pub = nh.advertise<visualization_msgs::Marker>("viz_ego_obb", 1);
    obs_obb_pub = nh.advertise<visualization_msgs::MarkerArray>("viz_obs_obb", 1);

    sub_obj = nh.subscribe("/target_tracks", 1, obsCallback);
    sub_gps = nh.subscribe("/gps", 1, gpsCallback);
    sub_imu = nh.subscribe("/imu", 1, imuCallback);
    sub_vel = nh.subscribe("/Ego_topic", 1, mainCallback);
    sub_cam = nh.subscribe("/lane/path", 1, laneCallback);
    

    ros::spin();

    return 0;
}