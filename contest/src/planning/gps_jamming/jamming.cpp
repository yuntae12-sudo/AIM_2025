# include "planning/gps_jamming/jamming.hpp"
# include "planning/localization/localization.hpp"

using namespace std;

double Deg2Rad (double deg) {
    double rad = deg * (pi / 180.0);
    return rad;
}

bool Is_Jamming(const morai_msgs::GPSMessage::ConstPtr& gps_msg) {
    if(gps_msg->latitude == 0.0 && gps_msg->longitude == 0.0 && gps_msg->altitude == 0.0) {
        return true;
    }
    return false;
}

double GpsJamming(const morai_msgs::GPSMessage::ConstPtr& gps_msg, const contest::LaneInfo::ConstPtr& path_msg) {
    if(Is_Jamming(gps_msg)) {
        if(!path_msg) return 0.0;

        double cam_angle_deg = path_msg->angle;
        double steer_angle = -Deg2Rad(cam_angle_deg);
        return steer_angle;
    }
    return -999.0;
}

