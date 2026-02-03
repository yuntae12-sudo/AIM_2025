#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

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
# include "planning/objective_function/objective_function.hpp"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

extern ros::Publisher waypoints_pub;
extern ros::Publisher candidate_paths_pub;
extern ros::Publisher best_path_pub;
extern ros::Publisher boundary_pub;
extern ros::Publisher ego_obb_pub;
extern ros::Publisher obs_obb_pub;

void publishWaypoints(const vector<egoPath_struc>& path);
void publishCandidates(const vector<Candidate_struct>& Candidate_vec, ros::Publisher& pub);
void publishBestPath(const Candidate_struct& best_candidate, ros::Publisher& pub);
void calculatePath(Candidate_struct& candidate, const egoPose_struc& start_pose);
void predictTrajectory(Candidate_struct& candidate, const egoPose_struc& current_pose);
void publishBoundaries(const vector<egoPath_struc>& in_bound, const vector<egoPath_struc>& out_bound);
void publishEgoOBB(const OBB& obb);
void publishObstacleOBBs(const vector<Obstacle_struct>& obstacles);

# endif