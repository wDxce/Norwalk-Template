#include "main.h"

/*
Defining Functions
*/

/*
Helper functions
*/

//Just faster than typing pros::delay(time);
extern void wait(const float& time);

//Sets all motors speeds to zero
extern void stop();

/*
Driving Functions
*/

//Calls the translate functions
extern void translate(/*Parmeters*/);

//Calls the turning functions
extern void turning(/*Parmeters*/);

//Calls the curve functions
extern void curve(/*Parmeters*/);

