#ifndef OBJECTIVE_FUNCTION_HPP
#define OBJECTIVE_FUNCTION_HPP

# include <iostream>
# include <string>
# include <fstream>
# include <iomanip>
# include <algorithm>
# include <ros/ros.h>
# include <cmath>
# include <vector>
# include <ros/package.h>

# include "planning/localization/localization.hpp"

# include <morai_msgs/GPSMessage.h>
# include <sensor_msgs/Imu.h> 
# include <morai_msgs/EgoVehicleStatus.h>
# include <morai_msgs/CtrlCmd.h>


# define _USE_MATH_DEFINES

using namespace std;

//================== 구조체 선언 ==================//
// struct Obstacle_struct {
//     double e;        // 동쪽 좌표
//     double n;        // 북쪽 좌표

//     double obs_vel_e;    // east velocity (m/s)
//     double obs_vel_n;    // north velocity (m/s)

//     double radius;   // 반경(m)
// };
struct Point {
    double e;
    double n;
};

struct OBB {
    Point vertices[4];
};

struct Candidate_struct {
    // [입력값]
    double v;                         // 선속도
    double t_p;                       // 예견 시간
    
    // [제어값]
    double steer_angle;               // Stanley 결과 (delta)
    double angular_velocity;          // 궤적 예측용 각속도 (중요!)

    // [점수] Score 구조체를 따로 빼지 않고 합치는 게 관리하기 편함
    double score_heading;
    double score_dist_obs;
    double score_vel;
    double score_dist_path;
    double total_score; // 최종 점수

    double min_dist_boundary;

    vector<egoPose_struc> predicted_path;
    vector<pair<double, double>> path_points;
};

struct VectorSpace {
    double min_v;
    double max_v;
    double min_w;
    double max_w;
};

struct RobotConstants {
    double max_speed_spec = 27.78;     // Va: 절대 최대 속도
    double max_yaw_rate_spec = 4.78;   // Va: 절대 최대 각속도

    double accel_lin = 3.75;           // Vc: 가속도
    double decel_lin = 8.82;
    double accel_ang = 0.64;
    double decel_ang = 1.50256;
    
    double wheelbase = 3.0;            // w 계산용 축거 (차량에 맞게 수정)
    double length = 4.635;
    double width = 1.892;
    double f_overhang = 0.845;
    double r_overhang = 0.79;

    double dt = 2.0;                   // 제어 주기
};

struct RobotState {
    double x, y, yaw, v, w;
};

extern vector<Candidate_struct> Candidate_vec; // 나중에 cpp에서 다시 메모리 할당
extern vector<Obstacle_struct> Obstacle_vec;

//=================== 전역 변수 선언 ===================//
extern double L_d;

const double v_min = 5.0;  // m/s
const double v_max = 30.0 / 3.6; // m/s
const double v_step = 1.0 / 3.6; // m/s

// [New] 예측 시간(t_p) 샘플링 파라미터
const double tp_min = 0.5;
const double tp_max = 1.5;
const double tp_step = 0.5; // 0.5초 단위 (0.5, 1.0, 1.5, 2.0 -> 총 4가지 경우)

// 평가 가중치 (Weights)
// const double W_HEADING = 5.0;
// const double W_DIST_OBS = 7.0;
// const double W_VEL = 2.0;
// const double W_PATH = 5.0;
// const double W_BOUNDARY = 5.0;

// const double W_HEADING = 6.0;
// const double W_DIST_OBS = 3.0;
// const double W_VEL = 1.0;
// const double W_PATH = 6.0;

const double W_HEADING = 5.0;
const double W_DIST_OBS = 3.0;
const double W_VEL = 1.0;
const double W_PATH = 5.0;


// 차량 제원
const double wheel_base = 3.0; // m
const double ego_radius = 1.5;  // m (안전 마진 포함)

// =================== 함수 선언 ===================//
void generateCandidates(vector<Candidate_struct>& Candidate_vec, const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose,
                        const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg, const RobotConstants& roboconsts,
                        const vector<Obstacle_struct>& Obstacle_vec, const vector<egoPath_struc>& in_boundary, const vector<egoPath_struc>& out_boundary);
double getDistanceToOBB(double px, double py, const Obstacle_struct& obs);
OBB GetEgoOBB(const morai_msgs::GPSMessage::ConstPtr& gps_msg, egoPose_struc& egoPose, RobotConstants& ego_spec);
OBB GetObsOBB (Obstacle_struct& obs_state);
void evaluateCandidates(vector<Candidate_struct>& Candidate_vec, const vector<Obstacle_struct>& Obstacle_vec,
                        const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose, const egoVelocity_struc& egoVelocity_struc,
                        const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg,
                        int search_start_idx);
double normalize_angle(double angle);
int getCurrentIndex(const std::vector<egoPath_struc>& path, const egoPose_struc& pose, int last_idx);
Candidate_struct selectBestCandidate(const vector<Candidate_struct>& Candidate_vec);


#endif