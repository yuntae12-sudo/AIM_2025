# include "planning/localization/localization.hpp"
# include "planning/objective_function/objective_function.hpp"


using namespace std;

int findClosestPoint (const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose) {
    int closest_index = 0;
    double min_d = -1.0;

    for (int i = 0; i < egoPath_vec.size(); i++) {
        double de = egoPath_vec[i].e - egoPose.current_e;
        double dn = egoPath_vec[i].n - egoPose.current_n;
        double d = hypot(de, dn);

        if (min_d == -1.0 || d < min_d) {
            min_d = d;
            closest_index = i;
        }
    }
    return closest_index;
}

int findWaypoint (const vector<egoPath_struc>& egoPath_vec, const egoPose_struc& egoPose, const double L_d) {
    if (egoPath_vec.empty()) {
        cout <<"경로를 못 찾았습니다" << endl;
        return -1;
    }

    // 1. 현재 차와 가장 가까운 경로상의 인덱스 찾기
    int closest_idx = findClosestPoint(egoPath_vec, egoPose);
    // 2. 그 인덱스부터 "앞으로"만 경로를 탐색
    for (int i = closest_idx; i < egoPath_vec.size(); ++i) {
        double de = egoPath_vec[i].e - egoPose.current_e;
        double dn = egoPath_vec[i].n - egoPose.current_n;
        double d = hypot(de, dn);

        // 3. L_d보다 멀리 있는 첫 번째 점을 찾으면 반환
        if (d > L_d) {
            return i;
        }
    }

    // 4. 만약 L_d보다 먼 점을 못찾으면 (경로 끝에 도달) 경로의 가장 마지막 점을 목표
    return egoPath_vec.size() - 1;
}