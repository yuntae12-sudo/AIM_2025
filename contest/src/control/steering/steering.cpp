# include "planning/localization/localization.hpp"
# include "planning/objective_function/objective_function.hpp"


using namespace std;


double getYawErr (const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose, const double L_d) {
    int target_idx = findWaypoint(egoPath_vec, egoPose, L_d);
    if (target_idx == -1) return 0.0; // 경로 없을 때 방어

    int next_idx = target_idx + 1;

    // 인덱스 범위 강제 제한 (0 ~ size-1)
    if (next_idx >= (int)egoPath_vec.size()) {
        next_idx = egoPath_vec.size() - 1;
        target_idx = max(0, next_idx - 1); // -2 대신 최소 0 보장
    }

    double de = egoPath_vec[next_idx].e - egoPath_vec[target_idx].e;
    double dn = egoPath_vec[next_idx].n - egoPath_vec[target_idx].n;
    
    // 만약 두 점이 너무 가까워 차이가 0이면 atan2가 이상해질 수 있음
    if (hypot(de, dn) < 1e-6) return 0.0; 

    double path_yaw = atan2(dn, de);
    double yaw_err = path_yaw - egoPose.current_yaw;

    // 각도 정규화
    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2 * M_PI;

    return yaw_err;
}

double getDistanceErr(const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose) {
    int target_idx = findClosestPoint(egoPath_vec, egoPose);
    if (target_idx == -1) return 0.0;

    int p1_idx, p2_idx;
    
    // 타겟 인덱스가 마지막 근처일 때
    if (target_idx >= (int)egoPath_vec.size() - 1) {
        p1_idx = max(0, target_idx - 1);
        p2_idx = target_idx;
    } else {
        p1_idx = target_idx;
        p2_idx = target_idx + 1;
    }

    egoPath_struc p_start = egoPath_vec[p1_idx];
    egoPath_struc p_end = egoPath_vec[p2_idx];

    // 2. 경로 벡터 (Vector A: p_start -> p_end) 계산
    double vecA_e = p_end.e - p_start.e;
    double vecA_n = p_end.n - p_start.n;

    // 3. 차량 벡터 (Vector B: p_start -> vehicle) 계산
    double vecB_e = p_start.e - egoPose.current_e;
    double vecB_n = p_start.n - egoPose.current_n;

    // 4. 외적 (A.e * B.n) - (A.n * B.e) 계산 : distance error의 부호를 결정하기 위함
    double cross_product_z = (vecA_e * vecB_n) - (vecA_n * vecB_e);

    // 5. 부호 결정 (왼쪽: +, 오른쪽: -)
    int sign = (cross_product_z > 0) ? 1 : -1;

    // 6. 오차의 크기 계산: 가장 가까운 점 (egoPath_vec[target_idx])까지의 절대 거리
    double de = egoPath_vec[target_idx].e - egoPose.current_e;
    double dn = egoPath_vec[target_idx].n - egoPose.current_n;
    double magnitude = hypot(de, dn);

    // 7. 부호 * 크기 반환
    return sign * magnitude;
}

double getSteeringAngle (const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose, const double L_d) {
    // 1. 횡오차 계산
    double distance_err = getDistanceErr(egoPath_vec, egoPose);

    // 2. 헤딩 오차 계산
    double yaw_err = getYawErr(egoPath_vec, egoPose, L_d);

    // 3. 스티어링 각도 계산 (Stanley 공식)
    double steer_angle = yaw_err + atan2(distance_err, L_d);

    return steer_angle;
}