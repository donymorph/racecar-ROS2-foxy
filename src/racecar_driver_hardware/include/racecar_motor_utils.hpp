#include <cmath>
#include <algorithm>
#include <cstdint>

namespace racecar_utils {

// Clamps a value within the provided min and max range.
template <typename T>
T clamp(const T& value, const T& min_value, const T& max_value) {
    return std::max(min_value, std::min(value, max_value));
}


// Calculates the PWM value for the servo given the steering position command.
uint16_t calculateServoPWM(double servo_position_command) {
    const double max_steering_angle = M_PI / 4;   // Max steering angle in radians (+/- 45 degrees)
    const uint16_t servo_pwm_neutral = 1566;
    const uint16_t servo_pwm_max_left = 2077;
    const uint16_t servo_pwm_max_right = 1055;

    // Clamp servo_position_command_ to [-max_steering_angle, +max_steering_angle]
    double clamped_command = clamp(servo_position_command, -max_steering_angle, max_steering_angle);

    // Map clamped_command (radians) to PWM value
    uint16_t servo_pwm = servo_pwm_neutral;
    if (clamped_command > 0) {
        // Right turn
        servo_pwm = servo_pwm_neutral + (clamped_command / max_steering_angle) * (servo_pwm_max_right - servo_pwm_neutral);
    } else if (clamped_command < 0) {
        // Left turn
        servo_pwm = servo_pwm_neutral + (clamped_command / -max_steering_angle) * (servo_pwm_max_left - servo_pwm_neutral);
    }
    // If clamped_command == 0, servo_pwm remains neutral

    return servo_pwm;
}

uint16_t calculateMotorPWM(double value) {
    const uint16_t motor_pwm_neutral = 1500;
    const uint16_t max_pwm_offset_forward = 150;   // Maximum PWM increase for forward motion
    const uint16_t max_pwm_offset_backward = 300;  // Maximum PWM decrease for backward motion

    static double max_positive_value = 0.0;
    static double max_negative_value = 0.0;

    uint16_t motor_pwm = motor_pwm_neutral;

    if (value > 0) {
        // Handle positive values (forward motion)
        if (value > max_positive_value) {
            max_positive_value = value;
            motor_pwm = motor_pwm_neutral + max_pwm_offset_forward;
        } else if (max_positive_value != 0) {
            double percentage = (value / max_positive_value) * max_pwm_offset_forward;
            motor_pwm = motor_pwm_neutral + static_cast<uint16_t>(percentage);
        }
    } else if (value < 0) {
        // Handle negative values (backward motion)
        if (value < max_negative_value) {
            max_negative_value = value;
            motor_pwm = motor_pwm_neutral - max_pwm_offset_backward;
        } else if (max_negative_value < 0) {
            double percentage = (value / max_negative_value) * max_pwm_offset_backward;
            motor_pwm = motor_pwm_neutral - static_cast<uint16_t>(percentage);
        }
    } else {
        // value == 0, neutral position
        motor_pwm = motor_pwm_neutral;
    }

    return motor_pwm;
}

//Calculates the PWM value for the motor given the traction velocity command.
uint16_t calculateMotorPWM2(double traction_velocity_command) {
    const double max_forward_speed = 1.0;        // Max forward speed (m/s)
    const double max_backward_speed = -1.0;      // Max backward speed (m/s)
    const uint16_t motor_pwm_neutral = 1500;
    const uint16_t motor_pwm_max_forward = 1550;
    const uint16_t motor_pwm_max_backward = 1280;

    // Clamp traction_velocity_command_ to [max_backward_speed, max_forward_speed]
    double clamped_command = clamp(traction_velocity_command, max_backward_speed, max_forward_speed);

    // Map clamped_command to PWM value
    uint16_t motor_pwm = motor_pwm_neutral;
    if (clamped_command > 0) {
        // Forward
        motor_pwm = motor_pwm_neutral + (clamped_command / max_forward_speed) * (motor_pwm_max_forward - motor_pwm_neutral);
    } else if (clamped_command < 0) {
        // Backward
        motor_pwm = motor_pwm_neutral + (clamped_command / max_backward_speed) * (motor_pwm_max_backward - motor_pwm_neutral);
    }
    // If clamped_command == 0, motor_pwm remains neutral

    return motor_pwm;
}

} // namespace racecar_utils
