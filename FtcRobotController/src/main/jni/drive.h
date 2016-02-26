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

#define sprocket_pitch_radius 3.13/2.0 //inches
#define encoderticks_per_inch sprocket_pitch_radius*encoderticks_per_radian
#define encoderticks_per_cm sprocket_pitch_radius*2.54*encoderticks_per_radian
#define acceptableAngleError 2

//constants
#define drive_gear_ratio (64.0/80.0)

float left_drive_theta = 0;
float left_drive_omega = 0;

float right_drive_theta = 0;
float right_drive_omega = 0;

float avg_drive_theta = 0;
float avg_drive_omega = 0;

void zeroDriveSensors()
{
    left_drive_theta = 0;
    left_drive_omega = 0;

    right_drive_theta = 0;
    right_drive_omega = 0;

    avg_drive_theta = 0;
    avg_drive_omega = 0;
}

void updateDriveSensors()
{
    if(left_drive_omega != left_drive_omega) left_drive_omega = 0;
    if(right_drive_omega != right_drive_omega) right_drive_omega = 0;
    lowpassFirstDerivativeUpdate(left_drive_encoder*radians_per_encodertick, &left_drive_theta, &left_drive_omega, 10);
    lowpassFirstDerivativeUpdate(right_drive_encoder*radians_per_encodertick, &right_drive_theta, &right_drive_omega, 10);
    avg_drive_theta = (left_drive_theta+right_drive_theta)/2.0;
    avg_drive_omega = (left_drive_omega+right_drive_omega)/2.0;
}

//Bounds and Smooths joystick values for better handling.

float raw_x;
float raw_y;

#define smoothConstant 0.6 //Set this to something the driver likes

v2f smoothJoysticks254Style(v2f stick)//TODO: Fix this
{
    v2f smoothed;
    raw_x = clamp(raw_x, -1, 1);//Clamp between -1 and 1
    raw_y = clamp(raw_y, -1, 1);
    smoothed.data[0] = sin((pi * smoothConstant * raw_x / 2.0) /
                           (pi / 2.0)); //Sin wave: https://www.desmos.com/calculator/4hd9ovg7el
    smoothed.data[1] = sin((pi * smoothConstant * raw_y / 2.0) / (pi / 2.0));
    return smoothed;//Give back smooth x and y values
}

#endif //DRIVE
