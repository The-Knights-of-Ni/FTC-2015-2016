#ifndef DRIVE
#define DRIVE

#include "robotics.h"
//TODO: Traction control
//TODO: D-PAD Turn macros (90, 180, 270, simulated shift)
//TODO: Delete this comment

//TODO:
float * pleft_drive;
#define left_drive (*pleft_drive)
float * pright_drive;
#define right_drive (*pright_drive)

float * pleft_drive_voltage;
#define left_drive_voltage (*pleft_drive_voltage)
float * pright_drive_voltage;
#define right_drive_voltage (*pright_drive_voltage)

float * pheading;
#define heading (*pheading)
int * pleft_drive_encoder;
#define left_drive_encoder (*pleft_drive_encoder)
int * pright_drive_encoder;
#define right_drive_encoder (*pright_drive_encoder)

#define sprocket_pitch_radius (3.13/2.0) //inches
#define encoderticks_per_inch (sprocket_pitch_radius*encoderticks_per_radian)
#define encoderticks_per_cm (sprocket_pitch_radius*2.54*encoderticks_per_radian)
#define acceptableAngleError 3

//constants
#define drive_gear_ratio (64.0/80.0)

#define drive_kv 1

#define drive_kp 0.25
#define drive_ki 0.0

#define drive_kslow 0.15

#define turn_kp 0.025 //0.1*(slider0/100.0)
#define turn_ki 0.05 //0.05  //0.1*(slider1/100.0)
#define turn_kd 0.0   //0.1*(slider2/100.0)

static const float left_drive_speed_threshold = 0.1;
static const float right_drive_speed_threshold = 0.1;

#define past_buffers_size 12
float past_left_drive_thetas[past_buffers_size] = {}; //looped buffer
float past_right_drive_thetas[past_buffers_size] = {}; //looped buffer
int current_drive_frame = 0;
int n_valid_left_drive_angles = 0;
int n_valid_right_drive_angles = 0;

//current state
float left_drive_theta = 0;
float left_drive_omega = 0;

float right_drive_theta = 0;
float right_drive_omega = 0;

float avg_drive_theta = 0;
float avg_drive_omega = 0;

float left_drive_hold_theta = 0;
float right_drive_hold_theta = 0;

bool8 left_drive_active = 0;
bool8 right_drive_active = 0;

float left_drive_compensation = 0; //TODO: this type of stabalization is used alot, maybe make the stuff it uses a struct
float right_drive_compensation = 0;

void zeroDriveSensors()
{
    current_drive_frame = 0;
    n_valid_left_drive_angles = 0;
    n_valid_right_drive_angles = 0;

    left_drive_theta = 0;
    left_drive_omega = 0;

    right_drive_theta = 0;
    right_drive_omega = 0;

    avg_drive_theta = 0;
    avg_drive_omega = 0;

    left_drive_hold_theta = 0;
    right_drive_hold_theta = 0;

    left_drive_active = 0;
    right_drive_active = 0;

    left_drive_compensation = 0; //TODO: this type of stabalization is used alot, maybe make the stuff it uses a struct
    right_drive_compensation = 0;
}

void updateDriveSensors()
{
    if(left_drive_omega != left_drive_omega) left_drive_omega = 0;
    if(right_drive_omega != right_drive_omega) right_drive_omega = 0;
    lowpassFirstDerivativeUpdate(left_drive_encoder*radians_per_encodertick, &left_drive_theta, &left_drive_omega, 10);
    lowpassFirstDerivativeUpdate(right_drive_encoder*radians_per_encodertick, &right_drive_theta, &right_drive_omega, 10);
    avg_drive_theta = (left_drive_theta+right_drive_theta)/2.0;
    avg_drive_omega = (left_drive_omega+right_drive_omega)/2.0;

    past_left_drive_thetas[current_drive_frame] = left_drive_theta;
    past_right_drive_thetas[current_drive_frame] = right_drive_theta;
    current_drive_frame = (current_drive_frame+1)%past_buffers_size;
    if(n_valid_left_drive_angles < past_buffers_size) n_valid_left_drive_angles++;
    if(n_valid_right_drive_angles < past_buffers_size) n_valid_right_drive_angles++;
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
