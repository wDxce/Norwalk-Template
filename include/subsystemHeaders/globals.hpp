#include "main.h"
#include <math.h> // For M_PI
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include "pros/imu.hpp"

// External controller object
extern pros::Controller master;

/* Robot Specifics */
const float wheelSizeMM = 0.0f; // Change this to the size of your wheels in millimeters
const float distanceBetweenWheels = 13.0f; // Change if needed, in mm
const float PI = 3.14159265358979323846f; // Pi constant for precision calculations

// Function to calculate distance traveled based on encoder ticks
float distanceTraveled(const float& encoderTicks);

/* Structures */

// PID struct for autonomous turning
struct targetPID {
    pros::IMU sensor;
    float integral;
    float lastError;

    // Returns value to spin motor according to PID adjustment
    const float PIDAdjust(const float& target);

    // Finds error in motor speed to target
    const float proport(const float& target, const float& speed);

    // Finds total value of accumulated error (integral)
    const float integrate(const float& error);

    // Finds immediate change in error (derivative)
    const float derive(const float& error);
};

// PID struct for user control
struct speedPID {
    pros::Motor storedMotor;
    float integral;
    float lastError;

    // Returns value to spin motor according to PID adjustment
    const float PIDAdjust(const float& target);

    // Finds error in motor speed to target
    const float proport(const float& target, const float& speed);

    // Finds total value of accumulated error (integral)
    const float integrate(const float& error);

    // Finds immediate change in error (derivative)
    const float derive(const float& error);
};

// Struct containing all drivebase motors
struct drivebaseMotorStore {
    speedPID fr;
    speedPID mr;
    speedPID rr;
    speedPID fl;
    speedPID ml;
    speedPID rl;
};

extern void setDrivebaseBrakeMode(pros::motor_brake_mode_e_t brakeMode);

// Stores all motors used on drivebase
extern drivebaseMotorStore driveB;

/* Defining Motors */
// Drivebase motors
extern pros::Motor fr;
extern pros::Motor mr;
extern pros::Motor rr;
extern pros::Motor fl;
extern pros::Motor ml;
extern pros::Motor rl;

/* Defining Sensors */
extern targetPID headingHold;
extern pros::IMU inertial;

/* Defining Pneumatics */
// Struct to make a piston toggleable by storing its current state
template <class item>
struct pistonToggle {
    item toggle;
    bool state = false;
};

// Creates pneumatic solenoid
extern pistonToggle<pros::ADIDigitalOut> pistonName; 