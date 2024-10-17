#include "main.h"

/*
Structures
*/

struct adjustedMotors{
    float fr;
    float mr;
    float rr;
    float fl;
    float ml;
    float rl;
};

/*
Helper Functions
*/

//Simple function to allow the motors to spin based off the input values, most speed will just be auto calculated
extern void spinMotors(const float& frSpd, const float& mrSpd, const float& rrSpd, const float& flSpd, const float& mlSpd, const float& rlSpd);

/*
Driver Funcions
*/

//This can control the calling for driving fwd & rev, also the driver control functions
extern void PIDMotorSet(const float& vertTar, const float& latTar);

//Calling for the use of the inertial sensor used to calculate the turning
const adjustedMotors inertialCalcPidMotors(const float& target);

//Calls the motor use that will be based around turning, with the inertial sensor
extern void inertialMotorSet(const float& target, const float& speed);

/*
PID Calculations
*/

//The overal PID calculations made in the code
const adjustedMotors calcPidMotors(const float& rightTar, const float& leftTar);