//#######################################################################################################
// Name:             pid_controller.cpp
// Purpose:          To provide PID control system for each motor/wheel
// Description:      to be used with esp32 firmware and micro-ros
// Related Files:         
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), previous_error(0), integral(0) {}

float PIDController::calculate(float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    previous_error = error;
    return (kp * error) + (ki * integral) + (kd * derivative);
}
