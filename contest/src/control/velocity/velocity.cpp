# include "planning/localization/localization.hpp"
# include "planning/objective_function/objective_function.hpp"

using namespace std;

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