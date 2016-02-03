#ifndef DRIVE
#define DRIVE

#include "robotics.h"
//TODO: Traction control
//TODO: Negative Inertia
//TODO: D-PAD Turn macros (90, 180, 270, simulated shift)
//TODO: Drive on course (turn by bearing)
//TODO: Stabilized driving (if we get slammed, we should correct)
//TODO: Delete this comment

//TODO:
float * pleft_drive;
#define left_drive (*pleft_drive)
float * pright_drive;
#define right_drive (*pright_drive)
float * pheading;
#define heading (*pheading)
int * pleft_drive_encoder;
#define left_drive_encoder (*pleft_drive_encoder)
int * pright_drive_encoder;
#define right_drive_encoder (*pright_drive_encoder)

#define threshold 0.

#define sprocket_pitch_radius 3.13 //Inches
#define encoderticks_per_inch sprocket_pitch_radius*encoderticks_per_radian
#define encoderticks_per_cm sprocket_pitch_radius*2.54*encoderticks_per_radian
#define acceptableAngleError 2

void deadZone(v2f &stick)
{
    float stick_norm = norm(stick);
    if (stick_norm < threshold)
    {
        stick.data[0] = 0;
        stick.data[1] = 0;
    }
    else
    {
        stick * ((stick_norm - threshold) / (1.0f - threshold)) /
        stick_norm;
    }//Remap non-deadzone to full range. Unnecessary if we can't move at 10% pwm
}

void squareDeadZone(v2f &stick)
{
    if (fabs(stick.data[0]) < threshold)
    {
        stick.data[0] = 0;
    }
    if (fabs(stick.data[1]) < threshold)
    {
        stick.data[1] = 0;
    }
}

#define Px_0 0
#define Px_1 0.2 //Set this to something the driver likes
#define Px_2 1.0
#define Py_0 0
#define Py_1 0.8 //Set this to something the driver likes
#define Py_2 1

//Bounds and Smooths joystick values for better handling.
void smoothJoysticks(v2f *stick) //TODO: Move to <robot_name>.h and have custom constants
{
    stick->data[0] = bound(stick->data[0], -1,
                           1);//Clamp between -1 and 1, might just build the deadzone into here
    stick->data[1] = bound(stick->data[1], -1, 1);
    stick->data[0] = (stick->data[0] < 0 ? -1 : 1) *
                     ((1 - stick->data[0]) * ((1 - stick->data[0]) * Px_0 + stick->data[0] * Px_1) +
                      stick->data[0] * ((1 - stick->data[0]) * Px_1 + stick->data[0] *
                                                                      Px_2)); //Quadratic Bezier, might want to make this a separate definition later
    stick->data[1] = (stick->data[1] < 0 ? -1 : 1) *
                     ((1 - stick->data[1]) * ((1 - stick->data[1]) * Py_0 + stick->data[1] * Py_1) +
                      stick->data[1] * ((1 - stick->data[1]) * Py_1 + stick->data[1] * Py_2));
}

float raw_x;
float raw_y;

#define smoothConstant 0.6 //Set this to something the driver likes

v2f smoothJoysticks254Style(v2f stick)//TODO: Fix this
{
    v2f smoothed;
    raw_x = bound(raw_x, -1, 1);//Clamp between -1 and 1
    raw_y = bound(raw_y, -1, 1);
    smoothed.data[0] = sin((pi * smoothConstant * raw_x / 2.0) /
                           (pi / 2.0)); //Sin wave: https://www.desmos.com/calculator/4hd9ovg7el
    smoothed.data[1] = sin((pi * smoothConstant * raw_y / 2.0) / (pi / 2.0));
    return smoothed;//Give back smooth x and y values
}

#endif //DRIVE
