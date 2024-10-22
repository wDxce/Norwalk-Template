#include "main.h"
#include "pros/motors.hpp"
#include "pros/imu.hpp"

// PID Values (Adjust these to the standards of your robot)
float kP = 0.00; // Proportional constant
float kI = 0.00; // Integral constant
float kD = 0.00; // Derivative constant
float kPI = 0.00; // Proportional constant for inertial PID
float kII = 0.00; // Integral constant for inertial PID
float kDI = 0.00; // Derivative constant for inertial PID
#define integralLimit 10 // Maximum value for the integral term to prevent windup

/* HELPER FUNCTIONS */

// Calculate the distance traveled based on encoder counts
float getDistanceTraveled() {
    float encoderCounts = driveB.fr.storedMotor.get_position(); // Get encoder counts from the front-right motor
    return distanceTraveled(encoderCounts); // Calculate and return distance traveled
}

// Spin all the drive motors with specified speeds
void spinMotors(const float& frSpd, const float& mrSpd, const float& rrSpd, const float& flSpd, const float& mlSpd, const float& rlSpd) {
    driveB.fr.storedMotor = frSpd;
    driveB.mr.storedMotor = mrSpd;
    driveB.rr.storedMotor = rrSpd;
    driveB.fl.storedMotor = flSpd;
    driveB.ml.storedMotor = mlSpd;
    driveB.rl.storedMotor = rlSpd;
}

/* PID Calculation Logic */

// Calculate PID value for a single motor
float calculatePID(speedPID& pid, const float& target) {
    float currentSpeed = pid.storedMotor.get_actual_velocity(); // Get current motor speed
    float error = target - currentSpeed; // Calculate error between target and current speed
    float integral = pid.integral + error; // Update integral term with error
    
    // Limit integral to prevent windup
    if (fabs(integral) > integralLimit) {
        integral = (integral > 0) ? integralLimit : -integralLimit;
    }
    
    float derivative = error - pid.lastError; // Calculate derivative term
    
    // Calculate output using PID formula
    float output = (kP * error) + (kI * integral) + (kD * derivative);
    pid.lastError = error; // Update last error for next cycle
    pid.integral = integral; // Store integral for future use
    return output; // Return calculated output
}

// Set motor speeds for driving forward and reverse
void PIDMotorSet(const float& vertTar, const float& latTar) {
    const float rightTar = vertTar + latTar; // Calculate target for right side motors
    const float leftTar = vertTar - latTar; // Calculate target for left side motors
    auto adjustedValues = calcPidMotors(rightTar, leftTar); // Get adjusted motor values using PID
    spinMotors(
        adjustedValues.fr, adjustedValues.mr, adjustedValues.rr, 
        adjustedValues.fl, adjustedValues.ml, adjustedValues.rl
    ); // Spin motors with adjusted values
}

// Set motor speeds for turning using the inertial sensor
void inertialMotorSet(const float& target, const float& speed) {
    auto adjustedValues = inertialCalcPidMotors(target); // Get adjusted motor values using inertial PID
    spinMotors(
        adjustedValues.fr * speed, adjustedValues.mr * speed, adjustedValues.rr * speed,
        adjustedValues.fl * speed, adjustedValues.ml * speed, adjustedValues.rl * speed
    ); // Spin motors with adjusted values scaled by speed
}

/* NEW PID CALCULATIONS */

// Calculate PID values for all drive motors
const adjustedMotors calcPidMotors(const float& rightTar, const float& leftTar) {
    const auto& temp = adjustedMotors{
    
    // Right side calculations
        driveB.fr.PIDAdjust(rightTar),
        driveB.mr.PIDAdjust(rightTar),
        driveB.rr.PIDAdjust(rightTar),
    
    // Left side calculations
        driveB.fl.PIDAdjust(leftTar),
        driveB.ml.PIDAdjust(leftTar),
        driveB.rl.PIDAdjust(leftTar),
    };
    return temp;
}

// Calculate PID values for turning using the inertial sensor
const adjustedMotors inertialCalcPidMotors(const float& target) {
    const float returnValue = headingHold.PIDAdjust(target); // Get PID adjustment from heading hold
    return adjustedMotors{
        -returnValue, -returnValue, -returnValue, 
        returnValue, returnValue, returnValue
    }; // Adjust motor values for turning
}

/* Speed PID Functionality */

// Adjust PID for speed control
const float speedPID::PIDAdjust(const float& target) {
    return calculatePID(*this, target); // Use calculatePID function to get adjusted value
}

/* Inertial PID Functionality */

// Adjust PID for inertial control
const float targetPID::PIDAdjust(const float& target) {
    const float position = sensor.get_rotation(); // Get current sensor rotation
    return proport(target, position); // Use proportion method for PID adjustment
}

// Proportion part of PID for inertial control
const float targetPID::proport(const float& target, const float& position) {
    float error = target - position; // Calculate error between target and current position
    if (fabs(error) > 95) { // Limit error to prevent large adjustments
        error = (error / fabs(error)) * 95;
    }
    return kPI * error + integrate(error); // Return proportional plus integral adjustment
}

// Integral part of PID for inertial control
const float targetPID::integrate(const float& error) {
    integral += error; // Add error to integral
    if (fabs(error) > integralLimit) { // Limit integral to prevent windup
        integral = 0;
    }
    if (fabs(integral) > 95) { // Limit integral value
        integral = integral / fabs(integral) * 95;
    }
    return kII * integral + derive(error); // Return integral plus derivative adjustment
}

// Derivative part of PID for inertial control
const float targetPID::derive(const float& error) {
    const float derivative = error - lastError;
    lastError = error; // Update lastError for next derivative calculation
    return kDI * derivative;
}
