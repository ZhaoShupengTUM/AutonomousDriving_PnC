#include "carla_shenlan_pid_controller/pid_controller.h"

#include <assert.h>
#include <iostream>
namespace shenlan {
namespace control {

PIDController::PIDController(const double kp, const double ki, const double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
}

// /**to-do**/ 实现PID控制
double PIDController::Control(const double error, const double dt) {
    assert(dt > 0 && "dt must be positive!!!");   
    
    proportional_part = kp_ * error;
    derivative_part = (error - previous_error_)/dt * kd_;
    
    integral_ += error*dt;
    integral_part =  integral_ * ki_;

    current_output = proportional_part + derivative_part + integral_part;

    return current_output;
}

// /**to-do**/ 重置PID参数
void PIDController::Reset() {
    kp_ = 0.0;
    ki_ = 0.0;
    kd_ = 0.0;
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = false;

    proportional_part = 0;
    integral_part = 0;
    derivative_part = 0;
    current_output = 0;
}

}    // namespace control
}    // namespace shenlan
