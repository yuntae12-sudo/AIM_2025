# include "planning/localization/localization.hpp"
# include "planning/objective_function/objective_function.hpp"


using namespace std;

// src/path/coner.cpp (또는 corner.cpp)

bool isCorner(const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose, int closest_idx) {
    // 1. 현재 위치 근처의 각도 (가까운 두 점 이용)
    double current_path_angle = atan2(egoPath_vec[closest_idx + 1].n - egoPath_vec[closest_idx].n, 
                                      egoPath_vec[closest_idx + 1].e - egoPath_vec[closest_idx].e);
    
    // 2. 좀 더 앞쪽(예: 15m 앞)의 각도 (미리 코너를 예측하기 위함)
    int look_ahead_idx = findWaypoint(egoPath_vec, egoPose, L_d); 
    double future_path_angle = atan2(egoPath_vec[look_ahead_idx].n - egoPath_vec[look_ahead_idx - 1].n, 
                                     egoPath_vec[look_ahead_idx].e - egoPath_vec[look_ahead_idx - 1].e);

    // 3. 두 지점 사이의 각도 차이 계산 (라디안)
    double diff = abs(future_path_angle - current_path_angle);
    if (diff > M_PI) diff = (2 * M_PI) - diff; // 각도 뒤집힘 방지

    // 4. 보정용 디버깅 출력 (라디안을 도(degree)로 바꿔서 출력)
    double diff_deg = diff * (180.0 / M_PI);
    
    std::cout << "[Corner Debug] Angle Diff: " << diff_deg << " deg" << std::endl;

    // 5. 추천하는 기준 숫자 (Threshold)
    double threshold_deg = 20.0; 
    double threshold_rad = threshold_deg * (M_PI / 180.0);

    return (diff > threshold_rad);
}