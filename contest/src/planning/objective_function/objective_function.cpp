# include "planning/localization/localization.hpp"
# include "planning/objective_function/objective_function.hpp"
#include "visualization/visualization.hpp" 

using namespace std;
vector<Candidate_struct> Candidate_vec;
vector<Obstacle_struct> Obstacle_vec;

VectorSpace GetVelocityVectorSpace(
    RobotState& state,
    const RobotConstants& consts,
    const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg) {

    // 1. 상태 업데이트 (ROS 메시지 -> RobotState)
    state.v = vel_msg->velocity.x; 

    // 2. 각속도(w) 계산 (Bicycle Model)
    double steering_rad = vel_msg->wheel_angle * (M_PI / 180.0);
    
    // 속도가 거의 0일 때 w가 튀는 것을 방지
    if (std::abs(state.v) < 0.01) {
        state.w = 0.0;
    } else {
        state.w = (state.v / consts.wheelbase) * std::tan(steering_rad);
    }

    // -----------------------------------------------------
    // Step 1: Vc (Dynamic Limits - 현재 가감속 능력)
    // -----------------------------------------------------
    double min_vd = state.v - consts.decel_lin * consts.dt;
    double max_vd = state.v + consts.accel_lin * consts.dt;

    double min_wd = state.w - consts.decel_ang * consts.dt;
    double max_wd = state.w + consts.accel_ang * consts.dt;

    // -----------------------------------------------------
    // Step 2: 교집합 (Va ∩ Vc) -> 최종 탐색 범위 결정
    // -----------------------------------------------------
    VectorSpace result_space; 

    // 선속도 교집합: [0 ~ max_spec] AND [min_vd ~ max_vd]
    result_space.min_v = max(0.0, min_vd);
    result_space.max_v = min(consts.max_speed_spec, max_vd);

    // 각속도 교집합: [-max_spec ~ +max_spec] AND [min_wd ~ max_wd]
    result_space.min_w = max(-consts.max_yaw_rate_spec, min_wd);
    result_space.max_w = min(consts.max_yaw_rate_spec, max_wd);

    return result_space; // 값 4개가 담긴 구조체 반환
}

// 정규화 함수
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// [Helper Function] 현재 내 차량이 경로의 몇 번째 점에 있는지 찾기
int getCurrentIndex(const vector<egoPath_struc>& path, const egoPose_struc& pose, int last_idx) {
    
    // 1. 탐색 범위 설정
    int start_search, end_search;

    if (last_idx == -1) { 
        // 처음 실행(초기화 상태): 전체 탐색
        start_search = 0;
        end_search = path.size();
    } else {
        // 주행 중: 아까 있던 곳(last_idx) 기준 앞뒤로 조금만 탐색 (최적화)
        // 예: 아까 100번에 있었으면, 90번 ~ 200번 사이만 뒤짐
        start_search = max(0, last_idx - 10); 
        end_search   = min((int)path.size(), last_idx + 150);
    }

    int closest_idx = -1;
    double min_dist = 1e9; // 아주 큰 값

    // 2. 거리 비교 루프
    for (int i = start_search; i < end_search; ++i) {
        double dx = path[i].e - pose.current_e;
        double dy = path[i].n - pose.current_n;
        double dist = dx*dx + dy*dy; // sqrt(hypot) 안 쓰고 제곱 비교가 더 빠름

        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // 예외 처리: 만약 경로에서 너무 멀어졌거나 못 찾았으면 이전 값 유지
    if (closest_idx == -1) return last_idx;
    
    return closest_idx;
}

double GetMinDistToBoundary(double current_e, double current_n, const vector<egoPath_struc>& boundary) {
    double min_dist_sq = 1e9;
    for (const egoPath_struc& pt : boundary) {
        double de = current_e - pt.e;
        double dn = current_n - pt.n;

        // 바운더리 전체를 보면 연산을 너무 많이 하기 때문에 10m 정도만 탐색
        if(fabs(de) > 10.0 || fabs(dn) > 10.0) continue;

        double dist_sq = sqrt(de * de + dn * dn);

        if(dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
        }
    }
    return sqrt(min_dist_sq);
}

bool BoundaryCheck (double x, double y, const vector<egoPath_struc>& boundary, bool is_left_line) {
    if (boundary.empty()) return false;

    // 1. 내 위치에서 가장 가까운 바운더리 점 찾기 (간단한 탐색)
    double min_dist = 1e9;
    int idx = 0;

    // (최적화 팁: 전체 탐색 대신 현재 인덱스 주변만 탐색하면 더 빠름)
    for (int i = 0; i < boundary.size() - 1; ++i) {
        double d = (boundary[i].e - x)*(boundary[i].e - x) + (boundary[i].n - y)*(boundary[i].n - y);
        if (d < min_dist) {
            min_dist = d;
            idx = i;
        }
    }

    // 2. 벡터 생성 (경계선 벡터 & 내 위치 벡터)
    double Ax = boundary[idx].e;
    double Ay = boundary[idx].n;
    double Bx = boundary[idx+5].e;
    double By = boundary[idx+5].n;

    double vec_line_x = Bx - Ax;
    double vec_line_y = By - Ay;
    double vec_point_x = x - Ax;
    double vec_point_y = y - Ay;

    // 3. 외적 (Cross Product) 계산
    double cp = (vec_line_x * vec_point_y) - (vec_line_y * vec_point_x);

    if (is_left_line) { // 왼쪽 벽
        if (cp > 0) return true; // 더 왼쪽으로 가면 OUT
    } else {            // 오른쪽 벽
        if (cp < 0) return true; // 더 오른쪽으로 가면 OUT
    }
    return false; // Inside (Safe)
}

void generateCandidates(vector<Candidate_struct>& Candidate_vec, const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose,
                        const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg, const RobotConstants& roboconsts,
                        const vector<Obstacle_struct>& Obstacle_vec, const vector<egoPath_struc>& in_boundary, const vector<egoPath_struc>& out_boundary) {
    Candidate_vec.clear();
    RobotState current_state;
    VectorSpace boundary = GetVelocityVectorSpace(current_state, roboconsts, vel_msg);
    
    // --- [1. 파라미터 설정] ---
    
    // 조향 샘플링 설정
    int num_steer_samples = 41; 
    
    double max_steer_range = 1.0; // 좌우 최대 라디안
    double steer_step = (num_steer_samples > 1) ? (max_steer_range * 2.0) / (num_steer_samples - 1) : 0;

    double base_steer = 0.0;

    // 미리 바운더리와 겹치는 후보군 생성하지 않기 위한 변수
    double l_rear = wheel_base * 0.5; // 후륜까지의 거리
    double ego_width = 1.892;    // ego 차량 width
    double half_width = ego_width / 2.0;
    double safe_dist = half_width + 0.2; // 사이드미러 같은 외부 길이 생각해서 0.2 버퍼 추가

    // --- [2. 후보군 생성 루프] ---

    // Loop 1: 속도 (Velocity)
    for (double v = v_min; v <= v_max; v += v_step) {
        
        // Loop 2: 예측 시간 (Prediction Time) -> 논문 방식 적용
        // 짧은 거리(0.5s)부터 긴 거리(2.0s)까지 다양한 길이의 경로 생성
        for (double tp = tp_min; tp <= tp_max; tp += tp_step) {

            // Loop 3: 조향 오프셋 (Steering Offset)
            for (int i = 0; i < num_steer_samples; ++i) {
                Candidate_struct candidate;
                candidate.total_score = 0.0; // 초기화
                

                // 조향 오프셋 계산 (중간 인덱스가 0.0이 되도록)
                double steer_offset = -max_steer_range + (i * steer_step);
                double candidate_steer = base_steer + steer_offset;


                // 차량 물리 한계 (Hard Constraint)
                if (candidate_steer > 0.6) candidate_steer = 0.6;
                if (candidate_steer < -0.6) candidate_steer = -0.6;

                // 각속도 계산 (Kinematic Bicycle Model)
                // w = (v / L) * tan(delta)
                double candidate_w = (v / wheel_base) * tan(candidate_steer);                
                
                if (v < boundary.min_v || v > boundary.max_v || candidate_w < boundary.min_w || candidate_w > boundary.max_w) {
                    continue;
                }

                bool is_safe_path = true;

                // // 장애물 구조체 안에 장애물이 있으면 바운더리 넘어가는 후보경로 삭제하는 로직
                //if(!Obstacle_vec.empty()) {
                    double future_e = egoPose.current_e;
                    double future_n = egoPose.current_n;
                    double future_yaw = egoPose.current_yaw;
                    double dt = 0.2;
                    for (double t = 0; t < tp; t += dt) {
                        double beta = atan((l_rear / wheel_base) * tan(candidate_steer));
                        future_e += v * cos(future_yaw + beta) * dt;
                        future_n += v * sin(future_yaw + beta) * dt;
                        future_yaw += (v * sin(beta) / l_rear) * dt;
                        bool out_left = BoundaryCheck(future_e, future_n, in_boundary_vec, true);
                        bool out_right = BoundaryCheck(future_e, future_n, out_boundary_vec, false);
                        // if (out_left && !out_right) {
                        //     is_ego_inside = true;
                        // }
                        // else if (!out_left && out_right) {
                        //     is_ego_inside = true;
                        // }
                        // else if (!out_left && !out_right) {
                        //     is_ego_inside = false;
                        // }
                        // else if (out_left && out_right) {
                        //     is_ego_inside = false;
                        // }
                        if (out_left || out_right) {
                            is_safe_path = false;
                        }
                    }
                //}
                 if (!is_safe_path) continue;

                // --- 통과된 후보만 등록 ---
                candidate.v = v;
                candidate.t_p = tp; 
                candidate.steer_angle = candidate_steer;
                candidate.angular_velocity = candidate_w;
                
                // 예측 경로 생성
                calculatePath(candidate, egoPose);
                
                Candidate_vec.push_back(candidate);
            }
        }
    }
}

void evaluateCandidates(vector<Candidate_struct>& Candidate_vec, const vector<Obstacle_struct>& Obstacle_vec,
                        const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose, const egoVelocity_struc& egoVelocity_struc,
                        const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg,
                        int search_start_idx) {
    double sum_score_heading = 0.0;
    double sum_score_dist_obs = 0.0;
    double sum_score_vel = 0.0;
    double sum_score_dist_path = 0.0;

    double l_rear = wheel_base * 0.5; // 후륜까지의 거리
    const double epsilon = 1e-6; // 작은 값으로 나누기 방지
    const double goal_v_none = 30.0 / 3.6; // 속도 제한값 (m/s)
    const double goal_v_obs = 20.0 / 3.6;  // 장애물 있을 시 mps
    double current_v = vel_msg->velocity.x;
    bool is_static = false;

    double max_dist_score = -1e9, min_dist_score = 1e9;
    double max_vel_score = -1e9, min_vel_score = 1e9;
    double max_head_score = -1e9, min_head_score = 1e9; 
    double max_path_score = -1e9, min_path_score = 1e9;
    double max_obs_vel = -1e9, min_obs_vel = 1e9;

    // ============================================================
    // [Loop 1] 시뮬레이션 & Raw Score 계산
    // ============================================================
    // 점수 계산 로직 구현 (예: heading, 거리, 속도 등)
    for (Candidate_struct& candidate : Candidate_vec) {
        // --- A. 미래 경로 예측 (Trajectory Prediction) ---
        // 현재 위치에서 시작
        double future_e = egoPose.current_e;
        double future_n = egoPose.current_n;
        double future_yaw = egoPose.current_yaw;
        double ego_vel_e = egoVelocity_struc.ego_vel_e;
        double ego_vel_n = egoVelocity_struc.ego_vel_n;

        double min_obs_dist = 100.0; // L_max: 장애물 없을 때 초기값
        double dt = 1.2;

        for (double t = 0; t < candidate.t_p; t += dt) {
            // 1. 내 차 이동 (Bicycle Model)
            double beta = atan((l_rear / wheel_base) * tan(candidate.steer_angle));

            future_e += candidate.v * cos(future_yaw + beta) * dt;
            future_n += candidate.v * sin(future_yaw + beta) * dt;
            future_yaw += (candidate.v * sin(beta) / l_rear) * dt;
            

            // 1. 장애물 충돌 체크 (정적, 동적 통합)
            for (const Obstacle_struct& obs : Obstacle_vec) {
                
                // 정적 장애물은 obs_vel_e, obs_vel_n이 0
                // double obs_future_e = obs.e + (obs.obs_vel_e * t);
                // double obs_future_n = obs.n + (obs.obs_vel_n * t);

                double dist = 0.0;
                double obs_radius = 1.5;

                if(abs(obs.obs_vel_e) < 2.0 || abs(obs.obs_vel_n) < 2.0) {
                    is_static = true;
                } else {
                    is_static = false;
                }

                // 거리 계산: (내 미래 위치) <-> (장애물 미래 위치) - (장애물 반지름)
                // 필요하다면 여기에 내 차의 반지름(혹은 안전마진)도 추가로 빼주면 더 안전합니다.
                // dist = hypot(future_e - obs_future_e, future_n - obs_future_n) - obs_radius - ego_radius;

                // 정적일 때
                if(is_static) {
                    double obs_future_e = obs.e;
                    double obs_future_n = obs.n;
                    dist = hypot(future_e - obs_future_e, future_n - obs_future_n) - obs_radius - ego_radius;

                    if(dist < min_obs_dist) {
                        min_obs_dist = dist;
                    }
                } else {
                    // 동적일 때
                    double obs_future_e = obs.e + (obs.obs_vel_e * t);
                    double obs_future_n = obs.n + (obs.obs_vel_n * t);
                    dist = hypot(future_e - obs_future_e, future_n - obs_future_n) - obs_radius - ego_radius;

                    if(dist < 6.0) {
                        candidate.total_score = -999;
                        break;
                    } else if (dist >= 6.0) {
                        if(dist < min_obs_dist) {
                            min_obs_dist = dist;
                        }
                    }
                }    
            }
        }       

        // 1. 충돌 판정 및 점수 기록
        if (min_obs_dist < 1.0) {
            candidate.total_score = -999.0; // 충돌 (즉시 탈락)
            continue; 
        }
        candidate.score_dist_obs = min_obs_dist;

        if (candidate.score_dist_obs > max_dist_score) {
            max_dist_score = candidate.score_dist_obs;
        }
        if (candidate.score_dist_obs < min_dist_score) {
            min_dist_score = candidate.score_dist_obs;
        }

        // 2. 속도 점수 기록
        // 수식: velocity(v, w) = v_i - v_actual
        // v_i: cand.v (후보 속도)
        // v_actual: current_v (현재 속도)
        // 의미: 현재 속도보다 더 빠를수록(가속할수록), 혹은 절대적으로 빠를수록 점수가 높음
        // if (!Obstacle_vec.empty()) {
        //     candidate.score_vel = -fabs(goal_v_none -candidate.v);
        // }
        // else {
        //     candidate.score_vel = -fabs(goal_v_obs -candidate.v);
        // }
        candidate.score_vel = -fabs(goal_v_none -candidate.v);





        if (candidate.score_vel > max_vel_score) {
            max_vel_score = candidate.score_vel;
        }
        if (candidate.score_vel < min_vel_score) {
            min_vel_score = candidate.score_vel;
        }

        // 3. 헤딩 점수 기록
        double L_d = candidate.v * candidate.t_p;
        L_d = std::min(L_d, 10.0); // 10m 이상 멀리 보지 않음
        if (L_d < 3.0) L_d = 3.0; // 너무 가까운 것도 방지

        int look_ahead_idx = findWaypoint(egoPath_vec, egoPose, L_d);
        int next_idx = look_ahead_idx + 1;
        double de = egoPath_vec[next_idx].e - egoPath_vec[look_ahead_idx].e;
        double dn = egoPath_vec[next_idx].n - egoPath_vec[look_ahead_idx].n;

        double waypoint_heading_angle = atan2(dn, de);
        double theta = fabs(normalize_angle(waypoint_heading_angle - future_yaw));
        if (hypot(de, dn) < 1e-6) theta = 0.0;

        candidate.score_heading = M_PI - theta;

        if (candidate.score_heading > max_head_score) {
            max_head_score = candidate.score_heading;
        }
        if (candidate.score_heading < min_head_score) {
            min_head_score = candidate.score_heading;
        }

        // 4. 경로추종 오차 점수 기록
        double min_path_dist = 1e9;
        
        // 최적화: 전체를 다 뒤지면 느리니까, 현재 인덱스부터 앞쪽 50개 정도만 검색
        int search_end = std::min((int)egoPath_vec.size(), search_start_idx + 100);
        
        for (int i = search_start_idx; i < search_end; ++i) {
            double d = hypot(future_e - egoPath_vec[i].e, future_n - egoPath_vec[i].n);
            if (d < min_path_dist) {
                min_path_dist = d;
            }
        }
        // 점수: 오차가 작을수록 커야 하므로 (-) 부호를 붙임
        candidate.score_dist_path = -min_path_dist;

        if (candidate.score_dist_path > max_path_score) max_path_score = candidate.score_dist_path;
        if (candidate.score_dist_path < min_path_score) min_path_score = candidate.score_dist_path;
    }
    // ============================================================
    // [Loop 2] Min, Max 정규화 & 가중치 합산
    // ============================================================
    
    double range_dist = max_dist_score - min_dist_score;
    double range_vel  = max_vel_score - min_vel_score;
    double range_head = max_head_score - min_head_score;
    double range_path = max_path_score - min_path_score;

    if (range_dist < epsilon) range_dist = 1.0;
    if (range_vel < epsilon)  range_vel  = 1.0;
    if (range_head < epsilon) range_head = 1.0;
    if (range_path < epsilon) range_path = 1.0;

    for (Candidate_struct& candidate : Candidate_vec) {
        
        if (candidate.total_score <= -900.0) continue;

        // 1. 정규화 (Normalization: 0.0 ~ 1.0)
        double norm_dist = (candidate.score_dist_obs - min_dist_score) / range_dist;
        double norm_vel  = (candidate.score_vel      - min_vel_score)  / range_vel;
        double norm_head = (candidate.score_heading  - min_head_score) / range_head;
        double norm_path = (candidate.score_dist_path- min_path_score) / range_path;

        // 2. 가중치 합산 (Weighted Sum)
        // W_PATH 상수를 추가해서 가중치를 조절하세요 (예: 0.5)
        candidate.total_score = (W_HEADING    * norm_head) + 
                           (W_VEL     * norm_vel)  +
                           (W_DIST_OBS * norm_dist) +
                           (W_PATH    * norm_path);
    }
}

Candidate_struct selectBestCandidate(const vector<Candidate_struct>& Candidate_vec) {
    
    Candidate_struct best_cand;
    
    // 1. 초기값 설정 (가장 낮은 점수로 초기화)
    best_cand.total_score = -1e9; 
    best_cand.v = 0.0;                  // 못 찾으면 정지
    best_cand.steer_angle = 0.0;
    
    bool found_valid = false;

    // 2. 1등 찾기 루프
    for (const Candidate_struct& cand : Candidate_vec) {
        
        // 충돌(-999)이나 비정상적인 경로는 패스
        if (cand.total_score <= -900.0) continue;

        // 현재 1등보다 점수가 높으면 갱신
        if (cand.total_score > best_cand.total_score) {
            best_cand = cand;
            found_valid = true;
        }
    }

    // 3. 예외 처리: 갈 수 있는 길이 하나도 없을 때! (All Blocked)
    if (!found_valid) {
        // 비상 정지 명령을 담은 더미 후보 리턴
        best_cand.v = 0.0;      
        best_cand.steer_angle = 0.0; // 혹은 이전 조향각 유지
        best_cand.total_score = -999.0; // 실패 표시
        
        // (디버깅용) 로그 출력
        cout << "[Warning] No valid path found! Emergency Stop." << endl;
    }

    return best_cand;
}