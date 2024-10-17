#include "main.h"

// Initialize controller
pros::Controller master(CONTROLLER_MASTER);

/*
Define Motors & Sensors
*/

// Drivebase Motors
#define frPort 1 // Change port to your ports
#define mrPort 2 // Change port to your ports
#define rrPort 3 // Change port to your ports
#define flPort 4 // Change port to your ports
#define mlPort 5 // Change port to your ports
#define rlPort 6 // Change port to your ports

// Sensors
#define inertialPort 7

// Pneumatics
#define pistonName 'a' // Change to your 3 wire port

/*
Initialize Drivebase motors
*/
// MOTOR_GEAR_ - This is the RPM of the motor alone
// RedMotor = 100, GreenMotor = 200, BlueMotor = 600
drivebaseMotorStore driveB = drivebaseMotorStore{
    // Right
    speedPID{pros::Motor(frPort, MOTOR_GEAR_600, false), 0, 0},
    speedPID{pros::Motor(mrPort, MOTOR_GEAR_600, false), 0, 0},
    speedPID{pros::Motor(rrPort, MOTOR_GEAR_600, false), 0, 0},

    // Left
    speedPID{pros::Motor(flPort, MOTOR_GEAR_600, true), 0, 0},
    speedPID{pros::Motor(mlPort, MOTOR_GEAR_600, true), 0, 0},
    speedPID{pros::Motor(rlPort, MOTOR_GEAR_600, true), 0, 0},
};

/*
Set Drivebase brake mode
*/
void setDriveBrakeMode(pros::motor_brake_mode_e_t brakeMode) {
    driveB.fr.storedMotor.set_brake_mode(brakeMode);
    driveB.mr.storedMotor.set_brake_mode(brakeMode);
    driveB.rr.storedMotor.set_brake_mode(brakeMode);
    driveB.fl.storedMotor.set_brake_mode(brakeMode);
    driveB.ml.storedMotor.set_brake_mode(brakeMode);
    driveB.rl.storedMotor.set_brake_mode(brakeMode);
}

/*
Distance Traveled
*/
float distanceTraveled(const float& encoderCounts) {
    float wheelCircumference = PI * wheelSizeMM; // Ensure wheelSizeMM is defined appropriately
    float rotations = encoderCounts / 360.0f; // Assuming 360 counts per rotation
    return rotations * wheelCircumference;
}

targetPID headingHold = targetPID{pros::IMU(inertialPort), 0, 0};
