//
// Created by Dev on 1/3/2016.
//

#ifndef FTCROBOTCONTROLLER_DRIVE_H
#define FTCROBOTCONTROLLER_DRIVE_H

#include "maths.h"
//TODO: Traction control
//TODO: Negative Inertia
//TODO: D-PAD Turn macros (90, 180, 270, simulated shift)
//TODO: Drive on course (turn by bearing)
//TODO: Stabilized driving (if we get slammed, we should correct)
//TODO: Delete this comment
typedef struct v2f joystick;

void scale(joystick *v, float s)
{
    v->data[0] *= s;
    v->data[1] *= s;
}

#define threshold 0.1
void deadZone(joystick *stick)
{
    float norm = (float)Math.sqrt(stick[0]*stick[0]+stick[1]*stick[1]);
    if(norm < threshold)
    {
        stick->data[0] = 0;
        stick->data[1] = 0;
    }
    else scale(stick, ((norm-threshold)/(1.0f-threshold))/norm);//Remap non-deadzone to full range. Unnecessary if we can't move at 10% pwm
}

void squareDeadZone(joystick *stick)
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

#define Px_0 0
#define Px_1 0.2 //Set this to something the driver likes
#define Px_2 1.0
#define Py_0 0
#define Py_1 0.8 //Set this to something the driver likes
#define Py_2 1
//Bounds and Smooths joystick values for better handling.
void smoothJoysticks(joystick *stick)
{
    stick->data[0] = bound(stick->data[0], -1, 1);//Clamp between -1 and 1, might just build the deadzone into here
    stick->data[1] = bound(stick->data[1], -1, 1);
    stick->data[0] = (stick->data[0] < 0 ? -1 : 1)*((1-stick->data[0])*((1-stick->data[0])*Px_0 + stick->data[0]*Px_1) +
            stick->data[0]*((1-stick->data[0])*Px_1 + stick->data[0]*Px_2)); //Quadratic Bezier, might want to make this a separate definition later
    stick->data[1] = (stick->data[1] < 0 ? -1 : 1)*((1-stick->data[1])*((1-stick->data[1])*Py_0 + stick->data[1]*Py_1) +
            stick->data[1]*((1-stick->data[1])*Py_1 + stick->data[1]*Py_2));
}
#define smoothConstant 0.6 //Set this to something the driver likes
float * smoothJoysticks254Style(float raw_x, float raw_y)
{
    v2f smoothed;
    raw_x = bound(raw_x, -1, 1);//Clamp between -1 and 1
    raw_y = bound(raw_y, -1, 1);
    smoothed.data[0] = sin((pi*smoothConstant*raw_x/2.0)/(pi/2.0)); //Sin wave: https://www.desmos.com/calculator/4hd9ovg7el
    smoothed.data[1] = sin((pi*smoothConstant*raw_y/2.0)/(pi/2.0));
    return smoothed.data;//Give back smooth x and y values
}
#endif //FTCROBOTCONTROLLER_DRIVE_H
