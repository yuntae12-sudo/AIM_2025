#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

# include <iostream>
# include <string>
# include <fstream>
# include <iomanip>
# include <algorithm>
# include <ros/ros.h>
# include <cmath>
# include <vector>
# include <ros/package.h>

# include <morai_msgs/GPSMessage.h>
# include <sensor_msgs/Imu.h> 
# include <morai_msgs/EgoVehicleStatus.h>
# include <morai_msgs/CtrlCmd.h>
# include <lidar_code/Track.h>

# define _USE_MATH_DEFINES

using namespace std;

//================== 구조체 선언 ==================//
struct egoPose_struc {double current_e, current_n, current_yaw;};

struct egoPath_struc {double e, n ,u;};

struct PoseHistory_struc {double e, n;};

extern vector<egoPath_struc> egoPath_vec;

// 바운더리 벡터 선언
extern vector<egoPath_struc> in_boundary_vec;
extern vector<egoPath_struc> out_boundary_vec;

struct egoVelocity_struc {double target_velocity, accel_input, brake_input, ego_vel_e, ego_vel_n;};

struct Obstacle_struct {
    int id;
    double e;
    double n;
    double prev_e, prev_n;
    double obs_vel_e;
    double obs_vel_n;
    double heading;
    double length;
    double width;
    double radius;
    double obs_speed;
    bool obs_status; // true: static, false: moving
};

//=================== 전역 변수 선언 ===================//
constexpr double pi = M_PI;
const double a = 6378137.0;           // WGS-84 타원체 장축 반경 (단위: m)
const double e = 0.006694379991;      // WGS-84 타원체 이심률 제곱

const double ref_lat = 37.238838359501933;
const double ref_lon = 126.772902206454901;
const double ref_alt = 0.0;

static double current_e = 0.0;
static double current_n = 0.0;


extern double k;    // look-forward gain
extern double gain_ld;
extern double gain_k;




// =================== 함수 선언 ===================//
// localization.hpp
void gps2Enu (const morai_msgs::GPSMessage::ConstPtr& gps_msg, egoPose_struc& egoPose);
void yawTf (const sensor_msgs::Imu::ConstPtr& imu_msg, egoPose_struc& egoPose);
bool loadPath();
bool loadBoundaries();

void bodyframe2Enu (const egoPose_struc& egoPose, egoVelocity_struc& egoVelocity, const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg);
void obsCordinate (const lidar_code::Track& track_data, Obstacle_struct& obstaclePose, const egoPose_struc& egoPose, const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg);
void obsVelocity (const lidar_code::Track& track_data, Obstacle_struct& obstaclePose, const egoPose_struc& egoPose, const egoVelocity_struc& egoVelocity, const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg);
bool is_obs_static(const Obstacle_struct& obs);

// followPoint.hpp
int findClosestPoint(const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose);
int findWaypoint(const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose, const double L_d);

// coner.hpp
bool isCorner(const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose, int closest_idx);

// steering.hpp
double getYawErr(const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose, const double L_d);
double getDistanceErr(const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose);
double getSteeringAngle (const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose, const double L_d);
// velocity.hpp
// void velocityControl (const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg,
//                       const egoPose_struc& egoPose, egoVelocity_struc& egoVelocity,
//                       const Obstacle_struct& obstacle, const vector<Obstacle_struct>& Obstacle_vec);
void velocityControl (const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg, const egoPose_struc& egoPose, egoVelocity_struc& egoVelocity);
#endif