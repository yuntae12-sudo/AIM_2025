#ifndef JAMMING_HPP
#define JAMMING_HPP

# include <iostream>
# include <ros/ros.h>
# include <cmath>
# include <ros/package.h>

# include <morai_msgs/GPSMessage.h>
# include "contest/LaneInfo.h"

# define _USE_MATH_DEFINES

using namespace std;

// 함수 선언

double Deg2Rad(double deg);
bool Is_Jamming(const morai_msgs::GPSMessage::ConstPtr& gps_msg);
double GpsJamming(const morai_msgs::GPSMessage::ConstPtr& gps_msg, const contest::LaneInfo::ConstPtr& path_msg);

#endif