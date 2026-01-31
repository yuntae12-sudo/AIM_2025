# include <ros/ros.h>
# include <stdio.h>
# include <iostream>
// 수정 부분 
# include <map>

# include "planning/localization/localization.hpp"
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

ros::Publisher control_pub;
ros::Publisher waypoints_pub;
ros::Publisher candidate_paths_pub;
ros::Publisher best_path_pub;
ros::Publisher boundary_pub;
ros::Publisher ego_circle_pub;

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
vector<egoPath_struc> egoPath_vec;
vector<egoPath_struc> in_boundary_vec;
vector<egoPath_struc> out_boundary_vec;

int last_closest_idx = 0;

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

void obsCallback (const lidar_code::TrackArray::ConstPtr& track_msg) {
    obstacles_map.clear();

    for (const auto& track_data : track_msg->tracks) {
    Obstacle_struct obstacle;

    obsCordinate(track_data, obstacle, egoPose);
    obsVelocity(track_data, obstacle, egoPose, egoVelocity);

    obstacle.length = track_data.length;
    obstacle.width = track_data.width;
    obstacle.heading = track_data.heading;

    // obstacle.id  = track.id;

    // // [수정된 코드] 장애물의 대각선 길이의 절반을 반지름으로 설정
    obstacle.radius = sqrt(pow(track_data.length, 2) + pow(track_data.width, 2)) / 2.0;

    // // 만약 값이 0이거나 너무 작으면 최소 크기 보정 (옵션)
    if (obstacle.radius < 0.1) obstacle.radius = 0.1;
    
    obstacles_map[track_data.id] = obstacle;
    }
}

void laneCallback (const contest::LaneInfo::ConstPtr& msg) {
    lane_msg = msg;
}

void mainCallback (const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg) {

    if(!gps_msg) return;
    // if(!lane_msg) return;
    
    bodyframe2Enu(egoPose, egoVelocity, vel_msg);
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

        generateCandidates(Candidate_vec, egoPath_vec, egoPose, vel_msg, roboconsts, obstacles, in_boundary_vec, out_boundary_vec);
        evaluateCandidates(Candidate_vec, obstacles, egoPath_vec, egoPose, egoVelocity, vel_msg, current_path_idx);

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
        printf("\n[DEBUG] Pos: (%.2f, %.2f) | Yaw: %.2f deg\n", egoPose.current_e, egoPose.current_n, egoPose.current_yaw * 180/M_PI);
        if (!egoPath_vec.empty()) {
            printf("        Idx: Close[%d], Target[%d] | Path_Pos: (%.2f, %.2f)\n", 
                closest_idx, target_idx, egoPath_vec[closest_idx].e, egoPath_vec[closest_idx].n);
        } else {
            printf("        Idx: Close[%d], Target[%d] | Path empty\n", closest_idx, target_idx);
        }
        printf("        Err: Dist[%.2f m], Yaw[%.2f deg] | Steer: %.2f deg\n", 
                dist_err, yaw_err * 180/M_PI, steering_angle * 180/M_PI);
        // [추가된 장애물 속도 출력 로직]
        if (!obstacles.empty()) {
            printf(" [OBSTACLE SPEED INFO]\n");
            int obs_count = 0;
            for (const auto& obs : obstacles) {
                double speed = hypot(obs.obs_vel_e, obs.obs_vel_n);
                // 상위 3개 장애물만 출력 (터미널 스크롤 방지)
                printf("        ID(Map): [%d] | Vel_E: %6.2f, Vel_N: %6.2f | Speed: %6.2f m/s\n", 
                        obs_count, obs.obs_vel_e, obs.obs_vel_n, speed);
                if (++obs_count >= 3) break;
            }
        } else {
            printf(" [OBSTACLE SPEED INFO] No obstacles detected.\n");
        }
            // 시각화 호출
        publishWaypoints(egoPath_vec);
        publishCandidates(Candidate_vec, candidate_paths_pub); 
        publishBestPath(best_candidate, best_path_pub);
        publishBoundaries(in_boundary_vec, out_boundary_vec);
        double check_radius = 1.5;
        publishEgoCircle(egoPose, check_radius);
    }

    velocityControl(vel_msg, egoPose, egoVelocity);

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
    ego_circle_pub = nh.advertise<visualization_msgs::Marker>("/rviz/ego_circle", 1);

    sub_obj = nh.subscribe("/target_tracks", 1, obsCallback);
    sub_gps = nh.subscribe("/gps", 1, gpsCallback);
    sub_imu = nh.subscribe("/imu", 1, imuCallback);
    sub_vel = nh.subscribe("/Ego_topic", 1, mainCallback);
    sub_cam = nh.subscribe("/lane/path", 1, laneCallback);
    

    ros::spin();

    return 0;
}