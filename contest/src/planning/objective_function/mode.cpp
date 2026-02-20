# include "planning/localization/localization.hpp"
# include "planning/objective_function/objective_function.hpp"

using namespace std;

namespace {
constexpr int kManualStartIdx = 159;
constexpr int kManualEndIdx = 349;
constexpr double kManualReleaseDist = 5.0; // m
constexpr int kPath01TargetVStartIdx = 405;
constexpr int kPath01TargetVEndIdx = 979;
bool g_manual_mode_latched = false;
int g_last_path01_idx = -1;

int getPath01ClosestIdx(const egoPose_struc& egoPose) {
    const auto it = path_library.find(1);
    if (it == path_library.end() || it->second.empty()) {
        return -1;
    }
    return findClosestPoint(it->second, egoPose);
}

double getPath01DistanceAtIdx(const egoPose_struc& egoPose, int idx) {
    const auto it = path_library.find(1);
    if (it == path_library.end() || it->second.empty()) return 1e9;
    if (idx < 0 || idx >= static_cast<int>(it->second.size())) return 1e9;

    const double de = it->second[idx].e - egoPose.current_e;
    const double dn = it->second[idx].n - egoPose.current_n;
    return hypot(de, dn);
}
} // namespace

bool obstacle_check(const Obstacle_struct& obs) {
    // 기준: 장애물 속도의 크기가 임계값보다 작으면 static
    const double speed_threshold_m_s = 50.0 * 3.6; // 0.5 m/s threshold (tunable)
    // double obs_speed = hypot(obs.obs_vel_e, obs.obs_vel_n);
    double obs_speed = obs.obs_speed; // 이미 계산된 속도 사용
    return obs_speed < speed_threshold_m_s;
}

void modeCheck (mode_struct& mode, const vector<Obstacle_struct>& Obstacle_vec, const egoPose_struc& egoPose) {
    mode.linear_mode = false;
    mode.coner_mode = false;
    mode.static_obstacle_mode = false;
    mode.dynamic_obstacle_mode = false;
    mode.manual_mode = false;

    int path01_idx = getPath01ClosestIdx(egoPose);
    if (path01_idx == -1) {
        // path_01을 못 찾는 경우에만 현재 active path로 fallback
        path01_idx = findClosestPoint(egoPath_vec, egoPose);
    }
    g_last_path01_idx = path01_idx;
    const double path01_dist = getPath01DistanceAtIdx(egoPose, path01_idx);

    if (!g_manual_mode_latched &&
        path01_idx >= kManualStartIdx &&
        path01_idx <= kManualEndIdx) {
        g_manual_mode_latched = true;
    }
    // 349 인덱스를 넘었더라도 path_01과 충분히 가까워졌을 때만 해제
    if (g_manual_mode_latched &&
        path01_idx > kManualEndIdx &&
        path01_dist < kManualReleaseDist) {
        g_manual_mode_latched = false;
    }
    if (g_manual_mode_latched) {
        mode.manual_mode = true;
        return;
    }

    int closest_idx = findClosestPoint(egoPath_vec, egoPose);
    bool coner_flag = isCorner(egoPath_vec, egoPose, closest_idx);
    if (coner_flag) {
        mode.coner_mode = true;
    } else if (!coner_flag) {
        mode.linear_mode = true;
    }

    for (const auto& obs : Obstacle_vec) {
        bool is_static = obstacle_check(obs);
        if (is_static) mode.static_obstacle_mode = true;
        else mode.dynamic_obstacle_mode = true;
    }
}

void manualMode (const mode_struct& mode, sampling_struct& sampling, Weight_struct& weight) {
    const int target_path_id = mode.manual_mode ? 2 : 1;
    if (mode.manual_mode) {
        // manual 모드에서는 직선 모드와 동일한 샘플링/가중치 사용
        sampling.v_min = 20.0 / 3.6;
        sampling.v_max = 50.0 / 3.6;
        sampling.v_step = 5.0 / 3.6;

        sampling.target_v = 40.0 / 3.6;

        sampling.tp_min = 0.5;
        sampling.tp_max = 2.0;
        sampling.tp_step = 0.5;

        weight.W_HEADING = 1.0;
        weight.W_DIST_OBS = 0.0;
        weight.W_VEL = 1.0;
        weight.W_PATH = 2.0;

        weight.LIMIT_PATH_ERR = 5.0;
        weight.LIMIT_HEADING = M_PI / 2.0;
        weight.LIMIT_VEL_ERR = 5.0 / 3.6;
        weight.LIMIT_DIST_OBS = 2.0;
    }

    if (getActivePathId() == target_path_id) return;

    if (!setActivePath(target_path_id)) {
        ROS_WARN_THROTTLE(1.0, "manualMode: failed to switch to path id=%d", target_path_id);
        return;
    }

    ROS_INFO_THROTTLE(1.0, "manualMode: switched active path to id=%d", target_path_id);
}

void linearMode (const mode_struct& mode, sampling_struct& sampling, Weight_struct& weight) {
    if (mode.linear_mode) {
        sampling.v_min = 20.0 / 3.6;
        sampling.v_max = 50.0 / 3.6;
        sampling.v_step = 5.0 / 3.6;

        sampling.target_v = 35.0 / 3.6; // m/s
        if (g_last_path01_idx >= kPath01TargetVStartIdx &&
            g_last_path01_idx <= kPath01TargetVEndIdx) {
            sampling.target_v = 20.0 / 3.6;
        }

        sampling.tp_min = 0.5;
        sampling.tp_max = 2.0;
        sampling.tp_step = 0.5;

        weight.W_HEADING = 1.0;
        weight.W_DIST_OBS = 0.0;
        weight.W_VEL = 1.0;
        weight.W_PATH = 2.0;

        weight.LIMIT_PATH_ERR = 5.0;
        weight.LIMIT_HEADING = M_PI / 2.0;
        weight.LIMIT_VEL_ERR = 10.0 / 3.6;
        weight.LIMIT_DIST_OBS = 2.0;
    }
}

void conerMode (const mode_struct& mode, sampling_struct& sampling, Weight_struct& weight) {
    if (mode.coner_mode) {
        sampling.v_min = 10.0 / 3.6;
        sampling.v_max = 40.0 / 3.6;
        sampling.v_step = 5.0 / 3.6;

        sampling.target_v = 20.0 / 3.6; // m/s
        if (g_last_path01_idx >= kPath01TargetVStartIdx &&
            g_last_path01_idx <= kPath01TargetVEndIdx) {
            sampling.target_v = 20.0 / 3.6;
        }

        sampling.tp_min = 0.5;
        sampling.tp_max = 2.0;
        sampling.tp_step = 0.5;

        weight.W_HEADING = 4.0;
        weight.W_DIST_OBS = 0.0;
        weight.W_VEL = 1.0;
        weight.W_PATH = 4.0;

        weight.LIMIT_PATH_ERR = 5.0;
        weight.LIMIT_HEADING = M_PI / 2.0;
        weight.LIMIT_VEL_ERR = 10.0 / 3.6;
        weight.LIMIT_DIST_OBS = 2.0;
    }
}

void staticObstacleMode (const mode_struct& mode, sampling_struct& sampling, Weight_struct& weight) {
    if (mode.static_obstacle_mode) {
        sampling.v_min = 10.0 / 3.6;
        sampling.v_max = 30.0 / 3.6;
        sampling.v_step = 2.0 / 3.6;

        sampling.target_v = 20.0 / 3.6; // m/s

        sampling.tp_min = 1.0;
        sampling.tp_max = 3.5;
        sampling.tp_step = 0.5;

        weight.W_HEADING = 0.7;
        weight.W_DIST_OBS = 2.0;
        weight.W_VEL = 0.7;
        weight.W_PATH = 1.2;

        weight.LIMIT_PATH_ERR = 5.0;
        weight.LIMIT_HEADING = M_PI / 2.0;
        weight.LIMIT_VEL_ERR = 5.0 / 3.6;
        weight.LIMIT_DIST_OBS = 2.0;
    }
}

void dynamicObstacleMode (const mode_struct& mode, sampling_struct& sampling, Weight_struct& weight, const Obstacle_struct& obs) {
    if (mode.dynamic_obstacle_mode) {
        sampling.v_min = 10.0 / 3.6;
        sampling.v_max = 30.0 / 3.6;
        sampling.v_step = 5.0 / 3.6;

        sampling.target_v = obs.obs_speed; // m/s (동적 장애물의 속도에 맞춤)

        sampling.tp_min = 0.5;
        sampling.tp_max = 2.0;
        sampling.tp_step = 0.5;

        weight.W_HEADING = 1.0;
        weight.W_DIST_OBS = 0.0;
        weight.W_VEL = 1.0;
        weight.W_PATH = 2.0;

        weight.LIMIT_PATH_ERR = 6.0;
        weight.LIMIT_HEADING = M_PI / 2.0;
        weight.LIMIT_VEL_ERR = 5.0 / 3.6;
        weight.LIMIT_DIST_OBS = 2.5;
    }
}
