# include "visualization/visualization.hpp"
# include <cmath>
# include "planning/localization/localization.hpp"
# include "planning/objective_function/objective_function.hpp"

using namespace std;

// [필수 추가] main.cpp에 있는 egoPose 변수를 가져옵니다. (빌드 에러 방지)
extern egoPose_struc egoPose;

// Safety limits to avoid publishing invalid geometry to RViz
static const double MAX_COORD_MAG = 1e9; 
static const size_t MAX_MARKER_POINTS = 1000; 

static size_t prev_waypoint_marker_count = 0;
static size_t prev_candidate_count = 0;
static size_t prev_best_marker_count = 0;
static size_t prev_in_bound_count = 0;
static size_t prev_out_bound_count = 0;
static size_t prev_obs_obb_count = 0;

// (calculatePath, predictTrajectory 로직은 원본 그대로 유지합니다.)
void calculatePath(Candidate_struct& candidate, const egoPose_struc& egoPose) {
    double sim_x = egoPose.current_e;
    double sim_y = egoPose.current_n;
    double sim_heading = egoPose.current_yaw;
    double v = candidate.v;
    double w = candidate.angular_velocity; 
    double dt = 0.1; 
    if (!std::isfinite(candidate.t_p) || candidate.t_p <= 0.0) candidate.t_p = dt;
    candidate.t_p = std::max(dt, std::min(candidate.t_p, 10.0));
    int max_steps = std::max(1, static_cast<int>(candidate.t_p / dt));
    candidate.path_points.clear(); 
    for (int i = 0; i < max_steps; ++i) {
        sim_x += v * cos(sim_heading) * dt;
        sim_y += v * sin(sim_heading) * dt;
        sim_heading += w * dt;
        if (std::isfinite(sim_x) && std::isfinite(sim_y) && std::abs(sim_x) < MAX_COORD_MAG && std::abs(sim_y) < MAX_COORD_MAG) {
            candidate.path_points.push_back(std::make_pair(sim_x, sim_y));
        } else break;
    }
}

void predictTrajectory(Candidate_struct& candidate, const egoPose_struc& current_pose) {
    double dt = 0.1; 
    int steps = candidate.t_p / dt;
    egoPose_struc temp_pose = current_pose;
    candidate.predicted_path.clear();
    for(int i = 0; i < steps; ++i) {
        temp_pose.current_e += candidate.v * cos(temp_pose.current_yaw) * dt;
        temp_pose.current_n += candidate.v * sin(temp_pose.current_yaw) * dt;
        temp_pose.current_yaw += candidate.angular_velocity * dt;
        candidate.predicted_path.push_back(temp_pose);
    }
}

// ----------------------------------------------------------------------------------
// 여기서부터 시각화 좌표 변환 수정본입니다.
// ----------------------------------------------------------------------------------

void publishWaypoints(const vector<egoPath_struc>& path) {
    const size_t chunk_size = MAX_MARKER_POINTS;
    if (path.empty()) {
        if (prev_waypoint_marker_count > 0) {
            for (size_t id = 0; id < prev_waypoint_marker_count; ++id) {
                visualization_msgs::Marker del;
                del.header.frame_id = "base_link"; // [수정]
                del.header.stamp = ros::Time::now();
                del.ns = "waypoints";
                del.id = static_cast<int>(id);
                del.action = visualization_msgs::Marker::DELETE;
                waypoints_pub.publish(del);
            }
            prev_waypoint_marker_count = 0;
        }
        return;
    }

    double cos_yaw = cos(egoPose.current_yaw);
    double sin_yaw = sin(egoPose.current_yaw);

    vector<geometry_msgs::Point> valid_pts;
    valid_pts.reserve(path.size());
    for (const auto &p : path) {
        if (!std::isfinite(p.e) || !std::isfinite(p.n)) continue;
        if (std::abs(p.e) >= MAX_COORD_MAG || std::abs(p.n) >= MAX_COORD_MAG) continue;
        
        // [수정] 로컬 좌표 변환
        double dx = p.e - egoPose.current_e;
        double dy = p.n - egoPose.current_n;
        geometry_msgs::Point pt; 
        pt.x = dx * cos_yaw + dy * sin_yaw; 
        pt.y = -dx * sin_yaw + dy * cos_yaw; 
        pt.z = 0.5;
        valid_pts.push_back(pt);
    }

    if (valid_pts.size() < 2) {
    // 점이 부족하면 발행하지 않고 함수를 종료합니다.
    return; 
    }

    size_t total_pts = valid_pts.size();
    size_t num_markers = (total_pts + chunk_size - 1) / chunk_size;
    size_t idx = 0;
    for (size_t m = 0; m < num_markers; ++m) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link"; // [수정]
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoints";
        marker.id = static_cast<int>(m);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0;
        marker.lifetime = ros::Duration(0.0);
        marker.points.clear();
        for (size_t j = 0; j < chunk_size && idx < total_pts; ++j, ++idx) {
            marker.points.push_back(valid_pts[idx]);
        }
        if (marker.points.size() >= 2) {
            waypoints_pub.publish(marker);
        }
    }

    if (num_markers < prev_waypoint_marker_count) {
        for (size_t id = num_markers; id < prev_waypoint_marker_count; ++id) {
            visualization_msgs::Marker del;
            del.header.frame_id = "base_link"; // [수정]
            del.header.stamp = ros::Time::now();
            del.ns = "waypoints";
            del.id = static_cast<int>(id);
            del.action = visualization_msgs::Marker::DELETE;
            waypoints_pub.publish(del);
        }
    }
    prev_waypoint_marker_count = num_markers;
}

void publishCandidates(const vector<Candidate_struct>& Candidate_vec, ros::Publisher& pub) {
    visualization_msgs::MarkerArray marker_array;
    size_t current_count = Candidate_vec.size();
    if (Candidate_vec.empty()) {
        if (prev_candidate_count > 0) {
            for (size_t i = 0; i < prev_candidate_count; ++i) {
                visualization_msgs::Marker del;
                del.header.frame_id = "base_link"; // [수정]
                del.header.stamp = ros::Time::now();
                del.ns = "candidates";
                del.id = i;
                del.action = visualization_msgs::Marker::DELETE;
                marker_array.markers.push_back(del);
            }
            if (pub) pub.publish(marker_array);
            else candidate_paths_pub.publish(marker_array);
            prev_candidate_count = 0;
        }
        return;
    }

    double cos_yaw = cos(egoPose.current_yaw);
    double sin_yaw = sin(egoPose.current_yaw);

    for (size_t i = 0; i < Candidate_vec.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link"; // [수정]
        marker.header.stamp = ros::Time::now();
        marker.ns = "candidates";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05; marker.color.a = 0.5;
        marker.color.r = 0.7; marker.color.g = 0.7; marker.color.b = 0.7;
        marker.lifetime = ros::Duration(0.0);

        for (const auto& pt_pair : Candidate_vec[i].path_points) {
            double dx = pt_pair.first - egoPose.current_e;
            double dy = pt_pair.second - egoPose.current_n;
            geometry_msgs::Point p;
            p.x = dx * cos_yaw + dy * sin_yaw;
            p.y = -dx * sin_yaw + dy * cos_yaw;
            p.z = 0.1;
            marker.points.push_back(p);
        }
        if (marker.points.size() >= 2){
        marker_array.markers.push_back(marker);
        }
    }

    if (current_count < prev_candidate_count) {
        for (size_t i = current_count; i < prev_candidate_count; ++i) {
            visualization_msgs::Marker del;
            del.header.frame_id = "base_link"; // [수정]
            del.header.stamp = ros::Time::now();
            del.ns = "candidates";
            del.id = i;
            del.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(del);
        }
    }
    if (pub) pub.publish(marker_array);
    else candidate_paths_pub.publish(marker_array);
    prev_candidate_count = current_count;
}

void publishBestPath(const Candidate_struct& best_candidate, ros::Publisher& pub) {
    const size_t chunk_size = MAX_MARKER_POINTS;
    if (best_candidate.path_points.empty()) {
        if (prev_best_marker_count > 0) {
            for (size_t id = 0; id < prev_best_marker_count; ++id) {
                visualization_msgs::Marker del;
                del.header.frame_id = "base_link"; // [수정]
                del.header.stamp = ros::Time::now();
                del.ns = "best_path";
                del.id = static_cast<int>(1000 + id);
                del.action = visualization_msgs::Marker::DELETE;
                if (pub) pub.publish(del); else best_path_pub.publish(del);
            }
            prev_best_marker_count = 0;
        }
        return;
    }

    double cos_yaw = cos(egoPose.current_yaw);
    double sin_yaw = sin(egoPose.current_yaw);

    vector<geometry_msgs::Point> pts;
    for (const auto &pt_pair : best_candidate.path_points) {
        double dx = pt_pair.first - egoPose.current_e;
        double dy = pt_pair.second - egoPose.current_n;
        geometry_msgs::Point p;
        p.x = dx * cos_yaw + dy * sin_yaw;
        p.y = -dx * sin_yaw + dy * cos_yaw;
        p.z = 0.2;
        pts.push_back(p);
    }

    if (pts.size() < 2) return;

    size_t total = pts.size();
    size_t num_markers = (total + chunk_size - 1) / chunk_size;
    size_t idx = 0;
    for (size_t marker_id = 0; marker_id < num_markers; ++marker_id) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link"; // [수정]
        marker.header.stamp = ros::Time::now();
        marker.ns = "best_path";
        marker.id = static_cast<int>(1000 + marker_id);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25; marker.color.a = 1.0;
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0.0);
        for (size_t j = 0; j < chunk_size && idx < total; ++j, ++idx) marker.points.push_back(pts[idx]);
        if (marker.points.size() >= 2) {
            if (pub) pub.publish(marker); else best_path_pub.publish(marker);
        }
    }

    if (num_markers < prev_best_marker_count) {
        for (size_t id = num_markers; id < prev_best_marker_count; ++id) {
            visualization_msgs::Marker del;
            del.header.frame_id = "base_link"; // [수정]
            del.header.stamp = ros::Time::now();
            del.ns = "best_path";
            del.id = static_cast<int>(1000 + id);
            del.action = visualization_msgs::Marker::DELETE;
            if (pub) pub.publish(del); else best_path_pub.publish(del);
        }
    }
    prev_best_marker_count = num_markers;
}

void publishSingleBoundaryChain(const vector<egoPath_struc>& path, string ns, size_t& prev_count, double r, double g, double b) {
    const size_t chunk_size = MAX_MARKER_POINTS;
    if (path.empty()) {
        if (prev_count > 0) {
            for (size_t id = 0; id < prev_count; ++id) {
                visualization_msgs::Marker del;
                del.header.frame_id = "base_link"; // [수정]
                del.header.stamp = ros::Time::now();
                del.ns = ns;
                del.id = static_cast<int>(id);
                del.action = visualization_msgs::Marker::DELETE;
                boundary_pub.publish(del);
            }
            prev_count = 0;
        }
        return;
    }

    double cos_yaw = cos(egoPose.current_yaw);
    double sin_yaw = sin(egoPose.current_yaw);

    vector<geometry_msgs::Point> valid_pts;
    valid_pts.reserve(path.size());
    for (const auto &p : path) {
        if (!std::isfinite(p.e) || !std::isfinite(p.n)) continue;
        if (std::abs(p.e) >= MAX_COORD_MAG || std::abs(p.n) >= MAX_COORD_MAG) continue;
        double dx = p.e - egoPose.current_e;
        double dy = p.n - egoPose.current_n;
        geometry_msgs::Point pt; 
        pt.x = dx * cos_yaw + dy * sin_yaw; 
        pt.y = -dx * sin_yaw + dy * cos_yaw; 
        pt.z = 0.5;
        valid_pts.push_back(pt);
    }

    if (valid_pts.size() < 2) return;

    size_t total_pts = valid_pts.size();
    size_t num_markers = (total_pts + chunk_size - 1) / chunk_size;
    size_t idx = 0;
    for (size_t m = 0; m < num_markers; ++m) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link"; // [수정]
        marker.header.stamp = ros::Time::now();
        marker.ns = ns; marker.id = static_cast<int>(m);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2; marker.color.a = 1.0;
        marker.color.r = r; marker.color.g = g; marker.color.b = b;
        marker.lifetime = ros::Duration(0.0);
        for (size_t j = 0; j < chunk_size && idx < total_pts; ++j, ++idx) marker.points.push_back(valid_pts[idx]);
        if (marker.points.size() >= 2) {
            boundary_pub.publish(marker);
        }
    }

    if (num_markers < prev_count) {
        for (size_t id = num_markers; id < prev_count; ++id) {
            visualization_msgs::Marker del;
            del.header.frame_id = "base_link"; // [수정]
            del.header.stamp = ros::Time::now();
            del.ns = ns;
            del.id = static_cast<int>(id);
            del.action = visualization_msgs::Marker::DELETE;
            boundary_pub.publish(del);
        }
    }
    prev_count = num_markers;
}

void publishBoundaries(const vector<egoPath_struc>& in_bound, const vector<egoPath_struc>& out_bound) {
    publishSingleBoundaryChain(in_bound, "in_boundary", prev_in_bound_count, 0.0, 0.0, 1.0);
    publishSingleBoundaryChain(out_bound, "out_boundary", prev_out_bound_count, 0.0, 0.0, 1.0);
}

void publishEgoOBB(const OBB& obb) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link"; // 다른 시각화와 동일하게 base_link 기준
    marker.header.stamp = ros::Time::now();
    marker.ns = "ego_obb";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    // 마커 설정 (두께 및 색상 - 주황색 추천)
    marker.scale.x = 0.15; 
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;

    double cos_yaw = cos(egoPose.current_yaw);
    double sin_yaw = sin(egoPose.current_yaw);

    // OBB의 4개 정점을 순회하며 local 좌표로 변환
    for (int i = 0; i < 4; ++i) {
        double dx = obb.vertices[i].e - egoPose.current_e;
        double dy = obb.vertices[i].n - egoPose.current_n;
        
        geometry_msgs::Point p;
        p.x = dx * cos_yaw + dy * sin_yaw;
        p.y = -dx * sin_yaw + dy * cos_yaw;
        p.z = 0.3; // 지면보다 살짝 높게 표시
        marker.points.push_back(p);
    }

    // 사각형을 닫기 위해 첫 번째 점을 마지막에 다시 추가
    if (!marker.points.empty()) {
        marker.points.push_back(marker.points[0]);
    }

    ego_obb_pub.publish(marker);
}

void publishObstacleOBBs(const vector<Obstacle_struct>& obstacles) {
    // 1. 장애물이 없으면 이전 마커들만 삭제하고 리턴
    if (obstacles.empty()) {
        if (prev_obs_obb_count > 0) {
            visualization_msgs::MarkerArray del_array;
            for (size_t i = 0; i < prev_obs_obb_count; ++i) {
                visualization_msgs::Marker del_marker;
                del_marker.header.frame_id = "base_link";
                del_marker.header.stamp = ros::Time::now();
                del_marker.ns = "obstacle_obbs";
                del_marker.id = static_cast<int>(i);
                del_marker.action = visualization_msgs::Marker::DELETE;
                del_array.markers.push_back(del_marker);
            }
            obs_obb_pub.publish(del_array);
            prev_obs_obb_count = 0;
        }
        return;
    }

    visualization_msgs::MarkerArray marker_array;

    // 2. 최대 그릴 개수를 50개로 제한 (버퍼 오버런 방지 핵심)
    size_t num_to_draw = min(obstacles.size(), (size_t)50);
    
    double cos_yaw = cos(egoPose.current_yaw);
    double sin_yaw = sin(egoPose.current_yaw);

    // [중요] 루프 범위를 obstacles.size()가 아닌 num_to_draw로 수정
    for (size_t i = 0; i < num_to_draw; ++i) {
        Obstacle_struct temp_obs = obstacles[i]; 
        
        // 데이터 유효성 검사
        if (!std::isfinite(temp_obs.e) || !std::isfinite(temp_obs.n)) continue;
        
        OBB obb = GetObsOBB(temp_obs);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacle_obbs";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.1); // 메시지 부하를 줄이기 위해 짧은 수명 설정

        marker.scale.x = 0.2; // 선 두께 적정 수준으로 조절 (0.5는 너무 굵을 수 있음)
        marker.color.a = 0.8;
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;

        for (int j = 0; j < 4; ++j) {
            double dx = obb.vertices[j].e - egoPose.current_e;
            double dy = obb.vertices[j].n - egoPose.current_n;

            geometry_msgs::Point p;
            p.x = dx * cos_yaw + dy * sin_yaw;
            p.y = -dx * sin_yaw + dy * cos_yaw;
            p.z = 0.3;
            
            // 좌표 변환 결과 유효성 재확인
            if (std::isfinite(p.x) && std::isfinite(p.y)) {
                marker.points.push_back(p);
            }
        }

        // 사각형이 성립될 때만 추가
        if (marker.points.size() >= 4) {
            marker.points.push_back(marker.points[0]);
            marker_array.markers.push_back(marker);
        }
    }

    // 3. 이전 프레임보다 장애물 수가 줄어들었을 경우, 남은 마커들 삭제
    if (num_to_draw < prev_obs_obb_count) {
        for (size_t i = num_to_draw; i < prev_obs_obb_count; ++i) {
            visualization_msgs::Marker del_marker;
            del_marker.header.frame_id = "base_link";
            del_marker.header.stamp = ros::Time::now();
            del_marker.ns = "obstacle_obbs";
            del_marker.id = static_cast<int>(i);
            del_marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(del_marker);
        }
    }

    // 4. 실제로 보낼 마커가 있을 때만 발행
    if (!marker_array.markers.empty()) {
        obs_obb_pub.publish(marker_array);
    }
    
    prev_obs_obb_count = num_to_draw;
}