# include "planning/localization/localization.hpp"
# include "planning/objective_function/objective_function.hpp"

using namespace std;

// void velocityControl (const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg,
//                       const egoPose_struc& egoPose, egoVelocity_struc& egoVelocity,
//                       const Obstacle_struct& obstacle, const vector<Obstacle_struct>& Obstacle_vec) {
//     double kp = 0.1;        // 바꿔야하는 변수~~!!!~!~!~
//     double ki = 0.0007;
//     double kd = 0.007;

//     double delta_time = 0.02;  // 1s
//     // -----------------------Gain + constant Variables--------------------------
//     // Candidate_struct best_candidate = selectBestCandidate(Candidate_vec);
//     // double target_velocity = best_candidate.v;
//     bool coner_flag = isCorner(egoPath_vec, egoPose, findClosestPoint(egoPath_vec, egoPose));

//     bool obstacle_detected = false;
//     ROS_INFO("velocityControl: Obstacle count = %zu", Obstacle_vec.size());
//     for(size_t i = 0; i < Obstacle_vec.size(); ++i) {
//        const auto& obs = Obstacle_vec[i];
//        double d = hypot(obs.e - egoPose.current_e, obs.n - egoPose.current_n);
//        if (i < 5) ;
//        if(d < 30.0) { // 30m 내에 장애물 있으면 감속 모드
//            obstacle_detected = true;
//            break;
//        }
//     }
//     // if (!obstacle_detected) ROS_INFO("velocityControl: obstacle_detected false");

//     bool is_obstacle_static = obstacle.obs_status; // true: static, false: moving
//     double target_velocity;
//     if (obstacle_detected) {
//         if (is_obstacle_static) {
//             target_velocity = 10.0 / 3.6; // m/s
//         } else {
//             target_velocity = 25.0 / 3.6; // m/s
//         }
//     }
//     else if (coner_flag) {
//         target_velocity = 30.0 / 3.6; // m/s
//     } 
//     else  {
//         target_velocity = 50.0 / 3.6; // m/s
//     }
//     double current_velocity = vel_msg->velocity.x;  // m/s
//     int closest_idx = findClosestPoint(egoPath_vec, egoPose);

//     // egoVelocity.target_velocity = target_velocity;
    
//     static double I_control_sum = 0.0;
//     static double last_error = 0.0;
//     static double prev_target_velocity = target_velocity;

//     if (target_velocity < prev_target_velocity - (3.0 / 3.6)) {
//         I_control_sum = 0.0;
//         last_error = 0.0;
//         ROS_WARN("Target velocity dropped sharply. Resetting I_control_sum.");
//     }
//     prev_target_velocity = target_velocity;


//     double error = target_velocity - current_velocity;      

//     double P_control = kp * error;
//     double tentative_signal = P_control + ki * I_control_sum + kd * (error - last_error) / delta_time;
//     bool is_saturated = (tentative_signal >= 1.0 || tentative_signal <= -1.0);
//     bool windup_direction = (tentative_signal > 0 && error > 0) || (tentative_signal < 0 && error < 0);

//     if (!is_saturated || !windup_direction) {
//         I_control_sum += error * delta_time;
//     }
//     double I_control = ki * I_control_sum;

//     double D_control = kd * (error - last_error) / delta_time;
//     last_error = error;

//     double control_signal = P_control + I_control + D_control;

//     // Clamp the control signal to [-1, 1]
//     double clamped_signal = max(-1.0, min(1.0, control_signal));

//     if (clamped_signal >= 0) {
//         egoVelocity.accel_input = clamped_signal;
//         egoVelocity.brake_input = 0.0;
//     } else {
//         egoVelocity.accel_input = 0.0;
//         egoVelocity.brake_input = abs(clamped_signal);  // 양수로 변환
//     }

//     cout << "target_velocity: " << target_velocity << endl;

// }

void velocityControl (const morai_msgs::EgoVehicleStatus::ConstPtr& vel_msg, const egoPose_struc& egoPose, egoVelocity_struc& egoVelocity) {
    double kp = 0.1;        // 바꿔야하는 변수~~!!!~!~!~
    double ki = 0.0007;
    double kd = 0.007;

    double delta_time = 0.02;  // 1s
    // -----------------------Gain + constant Variables--------------------------
    Candidate_struct best_candidate = selectBestCandidate(Candidate_vec);
    double target_velocity = best_candidate.v;
    double current_velocity = vel_msg->velocity.x;  // m/s
    int closest_idx = findClosestPoint(egoPath_vec, egoPose);

    egoVelocity.target_velocity = target_velocity;

    static double I_control_sum = 0.0;
    static double last_error = 0.0;

    double error = target_velocity - current_velocity;

    double P_control = kp * error;

    I_control_sum = I_control_sum + (error * delta_time);
    double I_control = ki * I_control_sum;

    double D_control = kd * (error - last_error) / delta_time;
    last_error = error;

    double control_signal = P_control + I_control + D_control;

    // Clamp the control signal to [-1, 1]
    double clamped_signal = max(-1.0, min(1.0, control_signal));

    if (clamped_signal >= 0) {
        egoVelocity.accel_input = clamped_signal;
        egoVelocity.brake_input = 0.0;
    } else {
        egoVelocity.accel_input = 0.0;
        egoVelocity.brake_input = abs(clamped_signal);  // 양수로 변환
    }

    cout << "target_velocity: " << target_velocity << endl;

}
