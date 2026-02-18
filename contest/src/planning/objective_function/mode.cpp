# include "planning/localization/localization.hpp"
# include "planning/objective_function/objective_function.hpp"

using namespace std;

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

void linearMode (const mode_struct& mode, sampling_struct& sampling, Weight_struct& weight) {
    if (mode.linear_mode) {
        sampling.v_min = 20.0 / 3.6;
        sampling.v_max = 50.0 / 3.6;
        sampling.v_step = 5.0 / 3.6;

        sampling.target_v = 40.0 / 3.6; // m/s

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
        weight.LIMIT_DIST_OBS = 5.0;
    }
}

void conerMode (const mode_struct& mode, sampling_struct& sampling, Weight_struct& weight) {
    if (mode.coner_mode) {
        sampling.v_min = 10.0 / 3.6;
        sampling.v_max = 30.0 / 3.6;
        sampling.v_step = 5.0 / 3.6;

        sampling.target_v = 20.0 / 3.6; // m/s

        sampling.tp_min = 0.5;
        sampling.tp_max = 2.0;
        sampling.tp_step = 0.5;

        weight.W_HEADING = 1.0;
        weight.W_DIST_OBS = 0.0;
        weight.W_VEL = 1.0;
        weight.W_PATH = 4.0;

        weight.LIMIT_PATH_ERR = 5.0;
        weight.LIMIT_HEADING = M_PI / 2.0;
        weight.LIMIT_VEL_ERR = 5.0 / 3.6;
        weight.LIMIT_DIST_OBS = 5.0;
    }
}

void staticObstacleMode (const mode_struct& mode, sampling_struct& sampling, Weight_struct& weight) {
    if (mode.static_obstacle_mode) {
        sampling.v_min = 5.0 / 3.6;
        sampling.v_max = 30.0 / 3.6;
        sampling.v_step = 5.0 / 3.6;

        sampling.target_v = 10.0 / 3.6; // m/s

        sampling.tp_min = 0.5;
        sampling.tp_max = 3.5;
        sampling.tp_step = 0.5;

        weight.W_HEADING = 0.05;
        weight.W_DIST_OBS = 2.0;
        weight.W_VEL = 1.3;
        weight.W_PATH = 0.05;

        weight.LIMIT_PATH_ERR = 6.0;
        weight.LIMIT_HEADING = M_PI / 2.0;
        weight.LIMIT_VEL_ERR = 5.0 / 3.6;
        weight.LIMIT_DIST_OBS = 2.5;
    }
}

void dynamicObstacleMode (const mode_struct& mode, sampling_struct& sampling, Weight_struct& weight, const Obstacle_struct& obs) {
    if (mode.dynamic_obstacle_mode) {
        sampling.v_min = 0.0 / 3.6;
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