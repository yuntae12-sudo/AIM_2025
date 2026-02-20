#ifndef OBSTACLE_TRACKER_HPP
#define OBSTACLE_TRACKER_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include "planning/localization/localization.hpp"

using namespace std;

// ============================================================================
// [Kalman Filter] 단순화된 2D 위치 추적기
// ============================================================================
struct KalmanFilter2D {
    // 상태: [x, y, vx, vy]
    double state[4];      // [e, n, ve, vn]
    double P[4][4];       // 공분산 행렬 (4x4)
    double Q[4][4];       // 프로세스 노이즈 행렬
    double R[2][2];       // 측정 노이즈 행렬
    double dt;            // 타임스텝
    
    KalmanFilter2D() : dt(0.1) {
        // 상태 초기화
        for (int i = 0; i < 4; i++) state[i] = 0.0;
        
        // 공분산 초기화 (P = I)
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
        
        // 프로세스 노이즈 (속도 변화 모델)
        // Q는 작게 (일정한 속도 가정) 
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                Q[i][j] = 0.0;
            }
        }
        Q[0][0] = 0.01; Q[1][1] = 0.01;  // 위치 변화
        Q[2][2] = 0.05; Q[3][3] = 0.05;  // 속도 변화
        
        // 측정 노이즈 (위치 센서만 있음)
        // 라이다 위치 측정 오차: ~0.1m
        R[0][0] = 0.05;  // E 방향
        R[1][1] = 0.05;  // N 방향
        R[0][1] = R[1][0] = 0.0;
    }
    
    // 초기화: 첫 측정값 설정
    void initialize(double e, double n, double ve, double vn) {
        state[0] = e;
        state[1] = n;
        state[2] = ve;
        state[3] = vn;
    }
    
    // [예측 단계] F*x + noise
    void predict() {
        double new_state[4];
        
        // 일정 속도 모델: x' = x + v*dt
        new_state[0] = state[0] + state[2] * dt;  // e' = e + ve*dt
        new_state[1] = state[1] + state[3] * dt;  // n' = n + vn*dt
        new_state[2] = state[2];                   // ve' = ve (상수)
        new_state[3] = state[3];                   // vn' = vn (상수)
        
        for (int i = 0; i < 4; i++) state[i] = new_state[i];
        
        // 공분산 업데이트: P' = F*P*F^T + Q
        // (간단화: P는 대각행렬 가정)
        P[0][0] += Q[0][0];
        P[1][1] += Q[1][1];
        P[2][2] += Q[2][2];
        P[3][3] += Q[3][3];
    }
    
    // [갱신 단계] 측정값으로 상태 보정
    void update(double z_e, double z_n) {
        // 측정 잔차 (innovation)
        double y_e = z_e - state[0];  // 위치 오차
        double y_n = z_n - state[1];
        
        // Kalman Gain 계산 (간단화)
        // K = P*H^T / (H*P*H^T + R)
        // H = [1, 0, 0, 0; 0, 1, 0, 0] (위치만 측정)
        double S_e = P[0][0] + R[0][0];
        double S_n = P[1][1] + R[1][1];
        
        double K_e = P[0][0] / S_e;
        double K_n = P[1][1] / S_n;
        
        // 상태 갱신
        state[0] += K_e * y_e;
        state[1] += K_n * y_n;
        
        // 공분산 갱신
        P[0][0] = (1.0 - K_e) * P[0][0];
        P[1][1] = (1.0 - K_n) * P[1][1];
    }
    
    // 현재 위치 반환
    void getPosition(double& e, double& n) const {
        e = state[0];
        n = state[1];
    }
    
    // 현재 속도 반환
    void getVelocity(double& ve, double& vn) const {
        ve = state[2];
        vn = state[3];
    }
};

// ============================================================================
// [TrackedObstacle] 추적 중인 장애물
// ============================================================================
struct TrackedObstacle {
    int id;
    KalmanFilter2D filter;
    int life_count;          // 프레임 생명력 (감지 안 되면 감소)
    int consecutive_frames;  // 연속 감지 프레임 수 (확실성)
    
    // 원본 센서 정보
    double width;
    double length;
    double heading;
    
    // [추가] 위치 고정 기능
    bool position_locked;    // true: 처음 위치 고정, false: 위치 갱신 가능
    double locked_e;         // 고정된 위치 (East)
    double locked_n;         // 고정된 위치 (North)
    
    TrackedObstacle() : id(-1), life_count(0), consecutive_frames(0), 
                        position_locked(false), locked_e(0.0), locked_n(0.0) {}
};

// ============================================================================
// [ObstacleTracker] 메인 추적기
// ============================================================================
class ObstacleTracker {
private:
    vector<TrackedObstacle> tracked_list;
    int next_id = 0;
    
    // 튜닝 파라미터
    const double MATCH_DIST_THRESHOLD = 2.0;  // [2m 이내] 같은 물체로 간주
    const int MAX_LIFE = 10;                  // [10프레임] 감지 안 돼도 유지
    const int MIN_CONFIRM = 2;                // [2프레임] 연속 감지 후 사용
    
public:
    ObstacleTracker() {}
    
    // 메인 루프에서 호출: 원본 센서 데이터 → 안정화된 위치
    vector<Obstacle_struct> updateObstacles(const vector<Obstacle_struct>& raw_obstacles) {
        
        vector<bool> raw_matched(raw_obstacles.size(), false);
        
        // [1단계] 예측: 기존 추적 목록 상태 예측
        for (auto& tracked : tracked_list) {
            tracked.filter.predict();
        }
        
        // [2단계] 데이터 연관: 센서 데이터 ↔ 기존 추적 목표 매칭 (최근접 이웃)
        for (auto& tracked : tracked_list) {
            double predicted_e, predicted_n;
            tracked.filter.getPosition(predicted_e, predicted_n);
            
            double min_dist = 1e9;
            int match_idx = -1;
            
            for (int j = 0; j < raw_obstacles.size(); ++j) {
                if (raw_matched[j]) continue;  // 이미 매칭됨
                
                double dist = hypot(predicted_e - raw_obstacles[j].e,
                                   predicted_n - raw_obstacles[j].n);
                
                if (dist < min_dist) {
                    min_dist = dist;
                    match_idx = j;
                }
            }
            
            // [3단계] 갱신 또는 생명력 감소
            if (match_idx != -1 && min_dist < MATCH_DIST_THRESHOLD) {
                // [수정] 위치가 고정되지 않은 경우만 Kalman Filter 갱신
                if (!tracked.position_locked) {
                    tracked.filter.update(raw_obstacles[match_idx].e, 
                                         raw_obstacles[match_idx].n);
                }
                // (위치가 고정되면 칼만 필터 업데이트 스킵)
                
                // 메타정보 업데이트
                tracked.width = raw_obstacles[match_idx].width;
                tracked.length = raw_obstacles[match_idx].length;
                tracked.heading = raw_obstacles[match_idx].heading;
                
                // 생명력 회복
                tracked.life_count = MAX_LIFE;
                tracked.consecutive_frames++;
                
                raw_matched[match_idx] = true;
            } else {
                // 매칭 실패 → 생명력 감소
                tracked.life_count--;
            }
        }
        
        // [4단계] 새로운 장애물 등록
        for (int j = 0; j < raw_obstacles.size(); ++j) {
            if (!raw_matched[j]) {
                TrackedObstacle new_obj;
                new_obj.id = raw_obstacles[j].id;  // [수정] 센서 원본 ID 보존
                new_obj.life_count = MAX_LIFE;
                new_obj.consecutive_frames = 1;
                
                // Kalman Filter 초기화
                new_obj.filter.initialize(raw_obstacles[j].e, 
                                         raw_obstacles[j].n, 
                                         0.0, 0.0);  // 초기 속도 0
                
                // [추가] 처음 감지된 위치를 고정
                new_obj.position_locked = true;
                new_obj.locked_e = raw_obstacles[j].e;
                new_obj.locked_n = raw_obstacles[j].n;
                
                // 메타정보
                new_obj.width = raw_obstacles[j].width;
                new_obj.length = raw_obstacles[j].length;
                new_obj.heading = raw_obstacles[j].heading;
                
                tracked_list.push_back(new_obj);
            }
        }
        
        // [5단계] 생명력 소진된 객체 제거
        tracked_list.erase(
            remove_if(tracked_list.begin(), tracked_list.end(),
                     [](const TrackedObstacle& t) { return t.life_count <= 0; }),
            tracked_list.end()
        );
        
        // [6단계] 결과 변환: 추적된 목표 → Obstacle_struct 벡터
        vector<Obstacle_struct> stable_obstacles;
        for (const auto& tracked : tracked_list) {
            // 최소 연속 프레임 확인 (깜빡임 방지)
            if (tracked.consecutive_frames < MIN_CONFIRM) continue;
            
            Obstacle_struct obs;
            double e, n, ve, vn;
            
            // [수정] 위치가 고정되면 고정된 위치 사용, 아니면 칼만 필터 위치 사용
            if (tracked.position_locked) {
                e = tracked.locked_e;
                n = tracked.locked_n;
            } else {
                tracked.filter.getPosition(e, n);
            }
            
            tracked.filter.getVelocity(ve, vn);
            
            obs.id = tracked.id;
            obs.e = e;
            obs.n = n;
            obs.obs_vel_e = ve;
            obs.obs_vel_n = vn;
            obs.width = tracked.width;
            obs.length = tracked.length;
            obs.heading = tracked.heading;
            obs.obs_speed = hypot(ve, vn) * 3.6;  // km/h
            
            stable_obstacles.push_back(obs);
        }
        
        return stable_obstacles;
    }
    
    // 디버깅용: 추적 중인 장애물 개수
    int getTrackedCount() const {
        return tracked_list.size();
    }
};

#endif // OBSTACLE_TRACKER_HPP
