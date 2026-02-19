//예측 기존 트랙들의 미래위치를 계산
//매칭 예측된 위치와 라이다 측정값을 비교(헝가리안)
//보정 매칭된 짝궁끼리 칼만 필터 업데이트
//관리 새 트랙 생성 및 죽은 트랙 삭제
#include <Lidar/ObjectTracking/ObjectTracking.hpp>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <chrono>

class LidarObjectTracker {
public:
    LidarObjectTracker() : last_timestamp(0.0), m_next_id(0) {}

    // 메인 프로세스 함수
    void process(LiDAR& st_LiDAR, double current_timestamp);

private:
    double last_timestamp;
    int m_next_id;

    std::vector<Lbox> m_prev_objects;

    // 내부 보조 함수들 
    float calculate2DIoU(const Lbox& obs, const Track& trk);
    void kalmanPredict(Track& t, double dt);
    void kalmanUpdate(Track& t, const Lbox& obs, const Eigen::Vector2f& y, const Eigen::Matrix2f& S);
};

// =============================================================================
// [보조 함수 구현부]
// =============================================================================

float LidarObjectTracker::calculate2DIoU(const Lbox& obs, const Track& t) {
    float obs_min_x = obs.f32_X_center - (obs.f32_L / 2.0f);
    float obs_max_x = obs.f32_X_center + (obs.f32_L / 2.0f);
    float obs_min_y = obs.f32_Y_center - (obs.f32_W / 2.0f);
    float obs_max_y = obs.f32_Y_center + (obs.f32_W / 2.0f);

    float trk_min_x = t.f32_X_center - (t.f32_L / 2.0f);
    float trk_max_x = t.f32_X_center + (t.f32_L / 2.0f);
    float trk_min_y = t.f32_Y_center - (t.f32_W / 2.0f);
    float trk_max_y = t.f32_Y_center + (t.f32_W / 2.0f);

    float inter_w = std::max(0.0f, std::min(obs_max_x, trk_max_x) - std::max(obs_min_x, trk_min_x));
    float inter_h = std::max(0.0f, std::min(obs_max_y, trk_max_y) - std::max(obs_min_y, trk_min_y));

    float intersection_area = inter_w * inter_h;
    if (intersection_area <= 0) return 0.0f;

    float union_area = (obs.f32_L * obs.f32_W) + (t.f32_L * t.f32_W) - intersection_area;
    return intersection_area / union_area;
}

void LidarObjectTracker::kalmanPredict(Track& t, double dt) {
    float f32_dt = static_cast<float>(dt);
    float f32_dt2 = 0.5f * f32_dt * f32_dt;

    Eigen::Matrix<float, 6, 6> F;
    F << 1, 0, f32_dt, 0, f32_dt2, 0,
         0, 1, 0, f32_dt, 0, f32_dt2,
         0, 0, 1, 0, f32_dt, 0,
         0, 0, 0, 1, 0, f32_dt,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Identity() * 0.1f;
    Q(0,0) = Q(1,1) = 0.5f * f32_dt; 
    Q(2,2) = Q(3,3) = 2.0f * f32_dt; 
    Q(4,4) = Q(5,5) = 20.0f * f32_dt; 

    t.vec_X = F * t.vec_X; 
    t.mat_P = F * t.mat_P * F.transpose() + Q;
}

void LidarObjectTracker::kalmanUpdate(Track& t, const Lbox& obs, const Eigen::Vector2f& diff, const Eigen::Matrix2f& S) {
    if (std::abs(S.determinant()) < 1e-6f) return;

    Eigen::Matrix<float, 2, 6> H;
    H.setZero(); 
    H(0, 0) = 1.0f; 
    H(1, 1) = 1.0f;

    Eigen::Matrix<float, 6, 2> K = t.mat_P * H.transpose() * S.inverse();
    t.vec_X = t.vec_X + K * diff; 
    t.mat_P = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * t.mat_P;

    t.f32_X_center = t.vec_X[0];
    t.f32_Y_center = t.vec_X[1];
    t.f32_Z_center = obs.f32_Z_center;
    t.f32_H = obs.f32_H;
}

// =============================================================================
// [메인 로직 구현부]
// =============================================================================

void LidarObjectTracker::process(LiDAR& st_LiDAR, double current_timestamp) {
    // 1. 시간차 계산
    if (last_timestamp == 0.0) { last_timestamp = current_timestamp; return; }
    double dt = current_timestamp - last_timestamp;
    if (dt <= 0 || dt > 1.0) {
        ROS_WARN("Time jump detected. Resetting tracker.");
        last_timestamp = current_timestamp; //비정상적인 시간 점프가 발생했지만 다음 프레임부터는 정상적으로 dt를 계산할수있도록 지금 타임 스탬프를 기준점으로 저장
        st_LiDAR.vec_Tracks.clear();
        m_next_id = 0;
        return;
    }
    last_timestamp = current_timestamp; //정상적인 프레임위한 다음 대비용


    // 2. Predict
    for (Track& st_Track : st_LiDAR.vec_Tracks) {
        kalmanPredict(st_Track, dt);
        st_Track.f32_X_center = st_Track.vec_X[0];
        st_Track.f32_Y_center = st_Track.vec_X[1];
        st_Track.s32_LostFrames++; //관측이 아직 안들어왔으므로 이 트랙은 1프레임동안 관측을 못받았다 표시
    }

    // 3. 매칭 및 [최적화 캐싱]
    size_t nObs = st_LiDAR.vec_Objects.size(); //.size의 반환타입이 size_t임(메모리 크기,컨테이너 크기, 인덱스 표현용), 음수가 없는 정수 타입이고 플랫폼에 따라 크기가 달라짐
    size_t nTracks = st_LiDAR.vec_Tracks.size();
    std::vector<int> assignments(nObs, -1);

    std::vector<std::vector<Eigen::Vector2f>> diff_cache(nObs, std::vector<Eigen::Vector2f>(nTracks)); //잔차,obs i와 track j사이얼마나 차이나는지
    std::vector<std::vector<Eigen::Matrix2f>> S_cache(nObs, std::vector<Eigen::Matrix2f>(nTracks)); //관측 예측 공분산
    std::vector<std::vector<float>> mahal_cache(nObs, std::vector<float>(nTracks)); //트랙의 예측 위치와 실제 관측간의 통계적 거리


    if (nObs > 0 && nTracks > 0) {
        HungarianAlgorithm solver;
        std::vector<std::vector<double>> cost_matrix(nObs, std::vector<double>(nTracks));

        // [개선] 게이트 강화: 40.0 -> 25.0 (더 엄격한 매칭 조건)
        // 이 값을 줄이면 거짓 매칭이 줄어들고 새 트랙 생성이 증가 (정확성 우선)
        const float GATE_CHI2 = 25.0f;

        for (size_t i = 0; i < nObs; ++i) {
            for (size_t j = 0; j < nTracks; ++j) {
                Track& t = st_LiDAR.vec_Tracks[j];
                const Lbox& obs = st_LiDAR.vec_Objects[i];

                Eigen::Vector2f z(obs.f32_X_center, obs.f32_Y_center); // 관측 = 센터

                Eigen::Matrix<float, 2, 6> H; 
                H.setZero(); 
                H(0,0) = 1.0f; 
                H(1,1) = 1.0f;

                Eigen::Vector2f diff = z - H * t.vec_X;
                
                // --- [개선] 측정 잡음 공분산 R 강화 ---
                // 센서 신뢰도를 높이기 위해 오차 범위를 축소
                Eigen::Matrix2f R;
                float sigma_x = 0.2f;   // x 방향 센터 오차 (m) 0.7 -> 0.2 (신뢰도 향상)
                float sigma_y = 0.2f;   // y 방향 센터 오차 (m)
                R.setZero();
                R(0,0) = sigma_x * sigma_x;
                R(1,1) = sigma_y * sigma_y; 

                Eigen::Matrix2f S = H * t.mat_P * H.transpose() + R;

                if (std::abs(S.determinant()) < 1e-6f) { 
                    cost_matrix[i][j] = 1e9; 
                    continue; 
                }

                float mahal_dist = diff.transpose() * S.inverse() * diff; // d^2



if (mahal_dist < 100.0f) {  // 너무 큰 값은 제외
    ROS_INFO_STREAM_THROTTLE(
        0.2,
        "[Mahal] d2=" << mahal_dist
        << " diff=(" << diff[0] << "," << diff[1] << ")"
        << " S=(" << S(0,0) << "," << S(1,1) << ")"
        << " trk_id=" << t.s32_ID
    );
}





                //  게이팅 
                if (mahal_dist > GATE_CHI2) {
                    cost_matrix[i][j] = 1e9;   // 매칭 불가
                    mahal_cache[i][j] = 1e9;
                    continue;
                }

                // 게이트 통과한 애들만 
                diff_cache[i][j]  = diff;
                S_cache[i][j]     = S;
                mahal_cache[i][j] = mahal_dist;

                // Likelihood 기반 비용
                // likelihood = exp(-0.5 * d^2)  (0~1, 클수록 좋은 매칭)
                float likelihood = std::exp(-0.5f * mahal_dist);

                // 수치 안정성 위해 [0,1] 범위로 한 번 더 클램프 
                if (likelihood < 0.0f) likelihood = 0.0f;
                if (likelihood > 1.0f) likelihood = 1.0f;

                float mahal_cost = 1.0f - likelihood;   // 0~1, 클수록 나쁜 매칭

                // IoU 기반 비용은 그대로
                float iou        = calculate2DIoU(obs, t);
                float iou_cost   = 1.0f - iou;          // 0~1, 안 겹칠수록 커짐

                // 두 기준 합치기 (가중치는 상황에 따라 조정 가능)
                cost_matrix[i][j] = 0.7f * mahal_cost + 0.3f * iou_cost;
            }
        }

        solver.Solve(cost_matrix, assignments); //assignments안에 들어감
    }


    // 4. Update & Create [메모리 안전 바구니 적용]
    std::vector<Track> new_tracks_to_add;
    for (size_t i = 0; i < nObs; ++i) {
        bool bMatched = false;
        int track_idx = assignments[i];

        if (track_idx >= 0 && track_idx < (int)nTracks) {
            Track& t = st_LiDAR.vec_Tracks[track_idx]; //헝가리안이 매칭하라고 선택한 j번 트랙 객체를 직접 가져오기
            const Lbox& obs = st_LiDAR.vec_Objects[i]; //i번째 관측된 물체 정보를 가져온다

            if (mahal_cache[i][track_idx] < 25.0f) {  // [개선] 임계값 일관성 (40.0 -> 25.0)
                bMatched = true;
                kalmanUpdate(t, obs, diff_cache[i][track_idx], S_cache[i][track_idx]);
                t.s32_LostFrames = 0;
                st_LiDAR.vec_Objects[i].s32_ID = t.s32_ID;

                // 비대칭 필터
                t.f32_Heading = obs.f32_Heading;

                    // 1. Length (길이): 더 큰 값이 들어오면(새로운 면 발견) 빠르게 확장
                    if (obs.f32_L > t.f32_L) t.f32_L = 0.5f * t.f32_L + 0.5f * obs.f32_L;
                    else                     t.f32_L = 0.4f * t.f32_L + 0.6f * obs.f32_L;

                    // 2. Width (폭): 위와 동일한 로직
                    if (obs.f32_W > t.f32_W) t.f32_W = 0.5f * t.f32_W + 0.5f * obs.f32_W;
                    else                     t.f32_W = 0.4f * t.f32_W + 0.6f * obs.f32_W;

                    // 3. Height (높이): 높이는 보통 안정적이지만, 일관성을 위해 동일 적용
                    if (obs.f32_H > t.f32_H) t.f32_H = 0.5f * t.f32_H + 0.5f * obs.f32_H;
                    else                     t.f32_H = 0.8f * t.f32_H + 0.2f * obs.f32_H;
            }
        }

        if (!bMatched) {
            const Lbox& obs = st_LiDAR.vec_Objects[i];

            // [개선] 새 트랙 생성 전 필터링: 너무 작은 클러스터는 무시
            // 가짓양성(False Positive) 감소
            if (obs.f32_L < 0.3f || obs.f32_W < 0.3f) continue;

            Track nt; 
            nt.vec_X.setZero();
            nt.mat_P = Eigen::Matrix<float, 6, 6>::Identity();

            nt.s32_ID         = m_next_id++;
            nt.s32_LostFrames = 0;

            // 상태를 센터로 초기화 (가장 중요)
            nt.vec_X[0] = obs.f32_X_center; // x_center
            nt.vec_X[1] = obs.f32_Y_center; // y_center

             // --- 속도 초기화: vx=(x-x_prev)/dt, vy=(y-y_prev)/dt ---
            float vx0 = 0.0f, vy0 = 0.0f;

            // dt 안전장치 (0 나눗셈 방지)
            const float fdt = static_cast<float>(dt);
            if (fdt > 1e-3f && !m_prev_objects.empty()) {
                float best_d2 = 1e9f;
                const Lbox* best_prev = nullptr;

                for (const auto& prev : m_prev_objects) {
                    float dx = obs.f32_X_center - prev.f32_X_center;
                    float dy = obs.f32_Y_center - prev.f32_Y_center;
                    float d2 = dx*dx + dy*dy;

                    if (d2 < best_d2) {
                        best_d2 = d2;
                        best_prev = &prev;
                    }
                }

                // 게이트(너무 멀면 같은 물체 아닐 확률↑) — 값은 상황에 맞게 튜닝
                const float GATE_DIST = 2.0f; // meters
                if (best_prev && best_d2 < GATE_DIST * GATE_DIST) {
                    float dx = obs.f32_X_center - best_prev->f32_X_center;
                    float dy = obs.f32_Y_center - best_prev->f32_Y_center;

                    vx0 = dx / fdt;
                    vy0 = dy / fdt;

                    // 속도 상한 (이상치 방지) — 대회 환경에 맞게 조절
                    const float VMAX = 50.0f; // m/s (180 km/h) 넉넉하게
                    vx0 = std::max(-VMAX, std::min(VMAX, vx0));
                    vy0 = std::max(-VMAX, std::min(VMAX, vy0));
                }
            }

            nt.vec_X[2] = vx0;
            nt.vec_X[3] = vy0;

            // nt.vec_X[2] = 0.0f;             // vx
            // nt.vec_X[3] = 0.0f;             // vy
            nt.vec_X[4] = 0.0f;             // ax
            nt.vec_X[5] = 0.0f;             // ay

            // 출력용 센터도 상태랑 맞춰두기
            nt.f32_X_center = nt.vec_X[0];
            nt.f32_Y_center = nt.vec_X[1];
            nt.f32_Z_center = obs.f32_Z_center;

            nt.f32_L       = obs.f32_L;
            nt.f32_W       = obs.f32_W;
            nt.f32_H       = obs.f32_H;
            nt.f32_Heading = obs.f32_Heading;

            nt.u8_R = rand() % 256;
            nt.u8_G = rand() % 256;
            nt.u8_B = rand() % 256;

            new_tracks_to_add.push_back(nt);
            st_LiDAR.vec_Objects[i].s32_ID = nt.s32_ID;
        }


    }
    for (const auto& nt : new_tracks_to_add) st_LiDAR.vec_Tracks.push_back(nt);

    // 5. Cleanup
    auto it = st_LiDAR.vec_Tracks.begin();
    while (it != st_LiDAR.vec_Tracks.end()) {
        if (it->s32_LostFrames > 7 || (it->vec_X.segment<2>(2).norm() * 3.6 > 150.0)) it = st_LiDAR.vec_Tracks.erase(it);
        else ++it;
    }

    // frame end: 다음 프레임 속도 초기화용
    m_prev_objects = st_LiDAR.vec_Objects;

}

// =============================================================================
// [Wrapper Function] L_ShapeFitting과 동일한 방식의 인터페이스
// =============================================================================
void ObjectTracking(LiDAR& st_LiDAR, double current_timestamp) {
    // static으로 선언하여 프로그램 실행 동안 단 하나의 트래커 객체만 유지 (m_next_id 등 보존)
    static LidarObjectTracker tracker; 

    // [1] 시작 시간 기록
    auto start = std::chrono::high_resolution_clock::now();


    tracker.process(st_LiDAR, current_timestamp);

    // [2] 종료 시간 기록
    auto end = std::chrono::high_resolution_clock::now();
    
    // [3] 실행 시간 및 FPS 계산
    std::chrono::duration<double, std::milli> elapsed = end - start;
    double ms = elapsed.count();
    double fps = 1000.0 / ms;

    // [4] 결과 출력 (10프레임마다 한 번씩)
    static int frame_count = 0;
    if (++frame_count % 10 == 0) {
        ROS_INFO("Tracking Time: %.2f ms | FPS: %.2f", ms, fps);
    }
    
}