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

OBB GetEgoOBB(const morai_msgs::GPSMessage::ConstPtr& gps_msg, egoPose_struc& egoPose, RobotConstants& ego_spec) {
    OBB obb;

    // [수정 1] 후륜 축(Rear Axle)에서 기하학적 중심(Geometric Center)까지의 거리 계산
    // 만약 ego_spec에 rear_overhang이 없다면: length - wheelbase - f_overhang 으로 계산 가능
    double dist_rear_to_geo_center = (ego_spec.length * 0.5) - ego_spec.r_overhang;

    // [수정 2] 후륜 기준이므로 중심은 차량 진행 방향(앞쪽)에 있음 -> 더하기(+) 연산
    // egoPose가 Rear Axle이라고 가정
    double center_e = egoPose.current_e + dist_rear_to_geo_center * cos(egoPose.current_yaw);
    double center_n = egoPose.current_n + dist_rear_to_geo_center * sin(egoPose.current_yaw);

    double half_l = ego_spec.length * 0.5;
    double half_w = ego_spec.width * 0.5;

    // Local coordinates: FL, FR, RR, RL (순서는 유지)
    double local_e[4] = { half_l,  half_l, -half_l, -half_l };
    double local_y[4] = { half_w, -half_w, -half_w,  half_w };

    double cos_y = cos(egoPose.current_yaw);
    double sin_y = sin(egoPose.current_yaw);

    for (int i = 0; i < 4; ++i) {
        // 회전 변환 (2D Rotation Matrix)
        // x' = x*cos - y*sin
        // y' = x*sin + y*cos
        obb.vertices[i].e = center_e + (local_e[i] * cos_y - local_y[i] * sin_y);
        obb.vertices[i].n = center_n + (local_e[i] * sin_y + local_y[i] * cos_y);
    }

    return obb;
}

OBB GetObsOBB (Obstacle_struct& obs_state) { 
    OBB obs_obb;

    // 데이터가 유효하지 않으면 0으로 초기화된 OBB 반환
    if (!std::isfinite(obs_state.e) || !std::isfinite(obs_state.n) || 
        !std::isfinite(obs_state.heading)) {
        for(int i=0; i<4; ++i) { obs_obb.vertices[i].e = 0; obs_obb.vertices[i].n = 0; }
        return obs_obb;
    }

    double half_l = obs_state.length * 0.5;
    double half_w = obs_state.width * 0.5;

    double local_e[4] = { half_l, half_l, -half_l, -half_l };
    double local_y[4] = { half_w, -half_w, -half_w, half_w };

    double cos_y = cos(obs_state.heading);
    double sin_y = sin(obs_state.heading);

    for (int i = 0; i < 4; ++i) {
        obs_obb.vertices[i].e = obs_state.e + (local_e[i] * cos_y - local_y[i] * sin_y);
        obs_obb.vertices[i].n = obs_state.n + (local_e[i] * sin_y + local_y[i] * cos_y);
    }

    return obs_obb;
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
    
    double max_steer_range = 0.45; // 좌우 최대 라디안
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

// ... (이전 include 및 전역변수들은 그대로 유지) ...

void evaluateCandidates(vector<Candidate_struct>& Candidate_vec, const vector<Obstacle_struct>& Obstacle_vec,
                        const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose, const egoVelocity_struc& egoVelocity_struc,
                        const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg,
                        int search_start_idx) {

    // [1] 절대 평가를 위한 물리적 한계값 정의 (Tuning Points)
    // 이 값들을 넘어서면 해당 항목 점수는 0점이 됩니다.
    const double LIMIT_PATH_ERR  = 8.0;        // 경로에서 3m 이상 벗어나면 0점 // 3.0 -> 5.0
    const double LIMIT_HEADING   = M_PI / 3.0; // 90도(1.57rad) 이상 틀어지면 0점
    const double LIMIT_VEL_ERR   = 5.0 / 3.6; // 목표 속도와 20km/h 이상 차이나면 0점
    const double LIMIT_DIST_OBS  = 5.0;       // 장애물이 20m보다 멀면 만점(1.0)

    // 목표 속도 설정
    // const double goal_v_noraml = 30.0 / 3.6;
    bool coner_flag = isCorner(egoPath_vec, egoPose, findClosestPoint(egoPath_vec, egoPose));
    double target_v = 10.0 / 3.6; // m/s
    // if (coner_flag) {
    //     target_v = 30.0 / 3.6; // m/s
    // } 
    // else  {
    //     target_v = 50.0 / 3.6; // m/s
    // }

    
    // 장애물 유무 판단
    //bool obstacle_detected = false;
    //for(const auto& obs : Obstacle_vec) {
    //    double d = hypot(obs.e - egoPose.current_e, obs.n - egoPose.current_n);
    //    if(d < 30.0) { // 30m 내에 장애물 있으면 감속 모드
    //        obstacle_detected = true;
    //        break;
    //    }
    //}
    // double target_v = obstacle_detected ? goal_v_obs : goal_v_normal;

    double l_rear = wheel_base * 0.5; // 후륜까지의 거리

    // ============================================================
    // [Loop] 시뮬레이션 & 절대 점수 계산 (One Pass)
    // ============================================================
    for (Candidate_struct& candidate : Candidate_vec) {
        
        // --- A. 미래 경로 예측 (Trajectory Prediction) ---
        double future_e = egoPose.current_e;
        double future_n = egoPose.current_n;
        double future_yaw = egoPose.current_yaw;
        
        double min_obs_dist = 100.0; // 초기값 (충분히 큰 값)
        double dt = 0.2;             // 시뮬레이션 dt (정밀도 필요시 조절)
        double obs_radius = 0.3;    // 장애물 반경 (고정값, 필요시 Obstacle_struct에 추가)

        // 시뮬레이션 루프
        for (double t = 0; t < candidate.t_p; t += dt) {
            // 1. 내 차 이동 (Kinematic Bicycle Model)
            double beta = atan((l_rear / wheel_base) * tan(candidate.steer_angle));
            future_e   += candidate.v * cos(future_yaw + beta) * dt;
            future_n   += candidate.v * sin(future_yaw + beta) * dt;
            future_yaw += (candidate.v * sin(beta) / l_rear) * dt;
            
            // 2. 장애물 거리 체크
            for (const Obstacle_struct& obs : Obstacle_vec) {
                // 간단화를 위해 정적 장애물 위치 사용 (동적 필요시 obs_vel 고려하여 t만큼 이동)
                // 내 차와 장애물 간의 유클리드 거리 - (장애물반경 + 내차반경)
                double dist = hypot(future_e - obs.e, future_n - obs.n) - obs_radius - ego_radius;
                // ROS_INFO("velocityControl: obstacle_detected true (obs id=%d, dist=%.2f)", obs.id, dist);

                if (dist < min_obs_dist) {
                    min_obs_dist = dist;
                }
            }
        }

        // 충돌 체크 (Hard Constraint)
        if (min_obs_dist < 0.3) { // 0.5m 이내면 충돌로 간주
            candidate.total_score = -999.0;
            continue; 
        }

        // ============================================================
        // [Score Calculation] 절대 평가 (Normalization 0.0 ~ 1.0)
        // ============================================================

        // 1. 장애물 점수 (Obstacle Score)
        // 멀수록 좋음. LIMIT_DIST_OBS 이상이면 1.0, 가까우면 0.0으로 선형 감소
        double norm_obs = 0.0;
        if (min_obs_dist >= LIMIT_DIST_OBS) {
            norm_obs = 1.0;
        } else if (min_obs_dist <= 0.0) {
            norm_obs = 0.0;
        } else {
            norm_obs = min_obs_dist / LIMIT_DIST_OBS;
        }

        
        // 2. 속도 점수 (Velocity Score)
        // 목표 속도와의 오차가 작을수록 좋음. LIMIT_VEL_ERR 이상 차이나면 0점
        double vel_err = fabs(target_v - candidate.v);
        double norm_vel = 1.0 - (vel_err / LIMIT_VEL_ERR);
        if (norm_vel < 0.0) norm_vel = 0.0; // Clamping

        // 3. 헤딩 점수 (Heading Score)
        // Lookahead 지점의 경로 방향과 내 차의 방향 오차 (작을수록 좋음)
        double L_d_val = std::max(3.0, std::min(candidate.v * candidate.t_p, 15.0));
        int look_ahead_idx = findWaypoint(egoPath_vec, egoPose, L_d_val);
        
        if (look_ahead_idx >= egoPath_vec.size() - 1) look_ahead_idx = egoPath_vec.size() - 2;

        double path_dx = egoPath_vec[look_ahead_idx+3].e - egoPath_vec[look_ahead_idx].e;
        double path_dy = egoPath_vec[look_ahead_idx+3].n - egoPath_vec[look_ahead_idx].n;
        double path_heading = atan2(path_dy, path_dx);

        double heading_err = fabs(normalize_angle(path_heading - future_yaw));
        double norm_head = 1.0 - (heading_err / LIMIT_HEADING);
        if (norm_head < 0.0) norm_head = 0.0;

        // 4. 경로 추종 점수 (Path Score)
        // 경로와의 최단 거리 오차 (작을수록 좋음)
        double min_path_dist = 1e9;
        int search_end = std::min((int)egoPath_vec.size(), search_start_idx + 400);
        
        // 간단한 탐색 (너무 멀면 LIMIT 처리되므로 적당히 찾음)
        for (int i = search_start_idx; i < search_end; ++i) {
            double d = hypot(future_e - egoPath_vec[i].e, future_n - egoPath_vec[i].n);
            if (d < min_path_dist) min_path_dist = d;
        }

        double norm_path = 1.0 - (min_path_dist / LIMIT_PATH_ERR);
        if (norm_path < 0.0) norm_path = 0.0;

        // ============================================================
        // [Total Score] 가중치 합산
        // ============================================================
        // 각 항목이 0.0 ~ 1.0 사이로 완벽히 정규화되었으므로 단순히 더하면 됨
        
        candidate.total_score = (W_HEADING  * norm_head) + 
                                (W_VEL      * norm_vel) + 
                                (W_DIST_OBS * norm_obs) + 
                                (W_PATH     * norm_path);

        // 디버깅용으로 개별 점수 저장 (필요시 사용)
        candidate.score_heading   = norm_head;
        candidate.score_vel       = norm_vel;
        candidate.score_dist_obs  = norm_obs;
        candidate.score_dist_path = norm_path;
    }
}

Candidate_struct selectBestCandidate(const vector<Candidate_struct>& Candidate_vec) {
    
    Candidate_struct best_cand;
    Obstacle_struct obs;
    
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