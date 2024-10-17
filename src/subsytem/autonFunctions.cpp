#include "main.h"

/*
Auton Functions
*/

/*
FUNCTION NOTES
--------------
The function in globals.cpp called distanceTraveled can be used in translate
Can also use forms of that logic and the distanceBetweenWheels in your turning
Using it in the turning Func can allow for double precision checking
*/

/*
Refer to Ike White for questions to creating functions
Try it out first to challenge yourself and improve as a programmer
Check out my github for updates: 
Go to this site to learn Purdue Pros: 
https://pros.cs.purdue.edu/v5/getting-started/new-users.html
*/

/*
Helper functions
*/

//Just a wait so its faster than pros::delay(time);
//Still in miliseconds so wait(1000); for 1 second
void wait(const float& time){
    pros::delay(time);
}

//Stop All Motors
void stop(){
    spinMotors(0, 0, 0, 0, 0, 0);
}

/*
Translate will be used to go forward and backwards
I suggest parmeters based around distance traveled and speed
Try to incorporate pi into this and have your wheel diameter defined for better precision
*/
void translate(){

}

/*
Creating a turning function that will turn based on the absolute value
I prefer using an absolute value turning function as you can do turning(90); and then turning(-45);
That will make it go 90 degrees right then 45 degrees left
*/
void turning(){

}

/*
Creating a curving function appears to be a lot more complex then it really is
In order to accomplish a curved drive it takes one side of the drivebase to move slower or faster
For consistency and precision try to incorporate the inertial sensor: moreconsistent stopping point
For parmeters I prefer to use left speed, and right speed, and target EX: curve(100,15,90);
That will make the curve go to the right because the left is moving faster and will stop at 90 degrees
*/
void curve(){

}