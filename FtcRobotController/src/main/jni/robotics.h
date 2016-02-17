/*
 * Robotics.h - Functions and constants for all programs.
 *
 *
*/

#ifndef ROBOTICS
#define ROBOTICS

#include "maths.h"
#include "jni_functions.h"
#include <stdlib.h>

//TODO: move these
float * pintake = 0;
#define intake (*pintake)

float * phand = 0;
#define hand (*phand)

float * pwrist = 0;
#define wrist (*pwrist)

float * phook_left = 0;
#define hook_left (*phook_left)

float * phook_right = 0;
#define hook_right (*phook_right)

int * pdim_digital_pins = 0;
#define dim_digital_pins (*pdim_digital_pins)
#define dim_digital_pin(n) ((dim_digital_pins>>(n))&1)

//Constants
#define encoderticks_per_radian (1440.0f/(2.0f*pi))

float potentiometer_range = 333.33333333333333333333333333333333333f;

#define continuous_servo_stop 0.5

//globals
float dt;
float current_time;

double * ptime;
#define time (*ptime)

#define min_motor_power 0.05
#define debuzz(a) (a)//(((a) < min_motor_power && (a) > -min_motor_power) ? 0 : (a))

#define deadzone_radius 0.1

float deadzoneAdjust(float a)
{
    if(a > deadzone_radius) return (a-deadzone_radius)/(1-deadzone_radius);
    if(a < -deadzone_radius) return (a+deadzone_radius)/(1-deadzone_radius);
    return 0;
}

v2f deadzone(v2f stick)
{
    float stick_norm = norm(stick);
    if (stick_norm < deadzone_radius)
    {
        stick.data[0] = 0;
        stick.data[1] = 0;
    }
    else
    {
        stick * ((stick_norm - deadzone_radius) / (1.0f - deadzone_radius)) /
        stick_norm;
    } //Remap non-deadzone to full range. Unnecessary if we can't move at 10% pwm
    return stick;
}

v2f squareDeadzone(v2f stick)
{
    if (fabs(stick.data[0]) < deadzone_radius)
    {
        stick.data[0] = 0;
    }
    if (fabs(stick.data[1]) < deadzone_radius)
    {
        stick.data[1] = 0;
    }
    return stick;
}

enum Colors
{
    BLUE,
    RED
};

#endif //ROBOTICS
