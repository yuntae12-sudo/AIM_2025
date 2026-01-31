# include <ros/ros.h>
# include <cmath>
# include <iostream>
# include <morai_msgs/GPSMessage.h>
# define _USE_MATH_DEFINES

using namespace std;



struct EgoPose {
    double current_e;
    double current_n;
    double current_u;
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

EgoPose egoPose = {0.0, 0.0};

void Gps2EnuCB (const morai_msgs::GPSMessage::ConstPtr& msg) {
    double wgs_lat = msg->latitude;
    double wgs_lon = msg->longitude;
    double wgs_alt = msg->altitude;
    
    // WGS값 radian으로 변경(계산 목적)
    double rad_lat = wgs_lat * pi / 180;
    double rad_lon = wgs_lon * pi / 180;
    double rad_alt = wgs_alt * pi / 180;
    double k = a / sqrt(1-e*pow(sin(rad_lat), 2));

    // WGS->ECEF
    double ecef_x = k * cos(rad_lat) * cos(rad_lon);
    double ecef_y = k * cos(rad_lat) * sin(rad_lon);
    double ecef_z = k * (1-e) * sin(rad_lat);

    double ref_rad_lat = ref_lat * pi / 180;
    double ref_rad_lon = ref_lon * pi / 180;
    double ref_rad_alt = ref_alt * pi / 180;
    double ref_k = a / sqrt(1-e*pow(sin(ref_rad_lat), 2));

    double ref_ecef_x = ref_k * cos(ref_rad_lat) * cos(ref_rad_lon);
    double ref_ecef_y = ref_k * cos(ref_rad_lat) * sin(ref_rad_lon);
    double ref_ecef_z = ref_k * (1-e) * sin(ref_rad_lat);


    // ecef 좌표 enu 좌표로 변환
    egoPose.current_e = (-sin(rad_lon)*(ecef_x - ref_ecef_x) + cos(rad_lon)*(ecef_y - ref_ecef_y));
    egoPose.current_n = (-sin(rad_lat)*cos(rad_lon)*(ecef_x - ref_ecef_x) - sin(rad_lat)*sin(rad_lon)*(ecef_y - ref_ecef_y) + cos(rad_lat)*(ecef_z - ref_ecef_z));
    egoPose.current_u = (cos(rad_lat)*cos(rad_lon)*(ecef_x - ref_ecef_x) + cos(rad_lat)*sin(rad_lon)*(ecef_y-ref_ecef_y) + sin(rad_lat)*(ecef_z-ref_ecef_z));


    cout << fixed;
    cout.precision(4); // 정밀도 높임
    cout << egoPose.current_e << " " 
         << egoPose.current_n << " " 
         << egoPose.current_u << endl;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "path_maker");
    ros::NodeHandle nh;

    ros::Subscriber gps_sub = nh.subscribe("/gps", 1, Gps2EnuCB);

    ros::spin();
    return 0;
}