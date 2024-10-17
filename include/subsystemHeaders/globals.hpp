#include "main.h"
#include <math.h> // To use M_PI for PI value

extern pros::Controller master;

/*
Robot Specifics
*/

const float wheelSizeMM = 0.0f; //Change this to the size of your wheels its in milimeters
const float distanceBetweenWheels = 13.0f;//Change if needed, in mm probably around 10-15
const float PI = 3.14159265358979323846f;//Can be used in auton functions for consistency/precision

float distanceTraveled(const float& encoderTicks);

/*
Structures
*/

//Struct in which controls the turning in Auton
struct targetPID{
    pros::IMU sensor;
    float integral;
    float lastError;
    //returns value to spin motor according to PID adjustment
    const float PIDAdjust(const float& target);
    //finds errro in motor speed to target
    const float proport(const float& target, const float& speed);
    //finds total value of accumulated error(integral)
    const float integrate(const float& error);
    //finds immediate change in error(derivative)
    const float derive(const float& error);
};

//Struct in which controls the User Control portion of PID
struct speedPID{
    pros::Motor storedMotor;
    float integral;
    float lastError;
    //returns value to spin motor according to PID adjusment
    const float PIDAdjust(const float& target);
    //finds error in motor speed to target
    const float proport(const float& target, const float& speed);
    //finds total value of accumulated error(integral)
    const float integrate(const float& error);
    //finds immediate change in error(derivative)
    const float derive(const float& error);
};

//Struct which contains all drivebase motors
struct drivebaseMotorStore{
    speedPID fr;
    speedPID mr;
    speedPID rr;
    speedPID fl;
    speedPID ml;
    speedPID rl;
};

//Stores all motors used on drivebase
extern drivebaseMotorStore driveB;


/*
Defining Motors
*/

//driveBase motors
extern pros::Motor fr;
extern pros::Motor mr;
extern pros::Motor rr;
extern pros::Motor fl;
extern pros::Motor ml;
extern pros::Motor rl;

/*
Defining Sensors
*/
extern targetPID headingHold;
extern pros::IMU inertial;

/*
Defining Pneumatics
*/

//Makes a piston toggleable by storing its current state
template <class item> struct pistonToggle{
    item toggle;

    bool state = false;
};

//Creates pnuematic colonoids
extern pistonToggle<pros::ADIDigitalOut> pistonName/*Can use that logic to define the piston*/;

/*
Set brake mode
*/
//Using this in auton will increase the consistency and brake point will be more precise
extern void setDriveBrakeMode(pros::motor_brake_mode_e_t brakeMode);



