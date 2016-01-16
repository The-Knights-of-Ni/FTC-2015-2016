#ifndef DRIVE
#define DRIVE

#include "robotics.h"
//TODO: Traction control
//TODO: Negative Inertia
//TODO: D-PAD Turn macros (90, 180, 270, simulated shift)
//TODO: Drive on course (turn by bearing)
//TODO: Stabilized driving (if we get slammed, we should correct)
//TODO: Delete this comment

#define threshold 0.1
void deadZone(v2f * stick)
{
    float norm = norm(stick);
    if(norm < threshold)
    {
        stick->data[0] = 0;
        stick->data[1] = 0;
    }
    else
        stick * ((norm-threshold)/(1.0f-threshold))/norm;//Remap non-deadzone to full range. Unnecessary if we can't move at 10% pwm
}

void squareDeadZone(v2f * stick)
{
    if(Math.abs(stick->data[0]) < threshold)
    {
        stick->data[0] = 0;
    }
    if(Math.abs(stick->data[1]) < threshold)
    {
        stick->data[1] = 0;
    }
}
//Returns change in drive pwm for a given target heading (add this vector to both drive pwms while driving forward)
v2f driveOnCourse(float target, float heading) //heading is relative to the driver box wall
{
    v2f drivePWMChange;
    float bearing = heading - target;
    if(bearing == 0)
    {
        drivePWMChange[0] = 0;
        drivePWMChange[1] = 0;
    }
    else if(bearing > 0)
    {
        drivePWMChange[0] = 1;
        drivePWMChange[1] = 1;
    }
    else if(bearing < 0)
    {
        drivePWMChange[0] = -1;
        drivePWMChange[1] = -1;
    }
    return drivePWMChange;
}

#define Px_0 0
#define Px_1 0.2 //Set this to something the driver likes
#define Px_2 1.0
#define Py_0 0
#define Py_1 0.8 //Set this to something the driver likes
#define Py_2 1
//Bounds and Smooths joystick values for better handling.
void smoothJoysticks(v2f * stick) //TODO: Move to <robot_name>.h and have custom constants
{
    stick->data[0] = bound(stick->data[0], -1, 1);//Clamp between -1 and 1, might just build the deadzone into here
    stick->data[1] = bound(stick->data[1], -1, 1);
    stick->data[0] = (stick->data[0] < 0 ? -1 : 1)*((1-stick->data[0])*((1-stick->data[0])*Px_0 + stick->data[0]*Px_1) +
            stick->data[0]*((1-stick->data[0])*Px_1 + stick->data[0]*Px_2)); //Quadratic Bezier, might want to make this a separate definition later
    stick->data[1] = (stick->data[1] < 0 ? -1 : 1)*((1-stick->data[1])*((1-stick->data[1])*Py_0 + stick->data[1]*Py_1) +
            stick->data[1]*((1-stick->data[1])*Py_1 + stick->data[1]*Py_2));
}
#define smoothConstant 0.6 //Set this to something the driver likes
v2f smoothJoysticks254Style(v2f stick)//TODO: Fix this
{
    v2f smoothed;
    raw_x = bound(raw_x, -1, 1);//Clamp between -1 and 1
    raw_y = bound(raw_y, -1, 1);
    smoothed.data[0] = sin((pi*smoothConstant*raw_x/2.0)/(pi/2.0)); //Sin wave: https://www.desmos.com/calculator/4hd9ovg7el
    smoothed.data[1] = sin((pi*smoothConstant*raw_y/2.0)/(pi/2.0));
    return smoothed.data;//Give back smooth x and y values
}
#endif //DRIVE
