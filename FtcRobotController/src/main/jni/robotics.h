/*
 * Robotics.h - Functions and constants for all programs.
 *
 *
*/

#ifndef ROBOTICS
#define ROBOTICS

#include "maths.h"

#ifndef USING_SIMULATOR
#include "jni_functions.h"
#endif

//TODO: move these
float *pintake = 0;
#define intake (*pintake)

float *phand = 0;
#define hand (*phand)

float *pwrist = 0;
#define wrist (*pwrist)

float *phook_left = 0;
#define hook_left (*phook_left)

float *phook_right = 0;
#define hook_right (*phook_right)

int *pdim_digital_pins = 0;
#define dim_digital_pins (*pdim_digital_pins)
#define dim_digital_pin(n) ((dim_digital_pins>>(n))&1)

//Constants
#define lbs 453.592
#define cm 2.54

#define robot_m (30*lbs) //robot mass in grams
#define robot_I (robot_m*sq(sprocket_pitch_radius))

#define encoderticks_per_radian (1440.0f/(2.0f*pi))
#define radians_per_encodertick (1.0/encoderticks_per_radian)

float potentiometer_range = 333.33333333333333333333333333333333333f;

float neverest_max_torque = 4334000; //in g in^2/s^2
float neverest_max_omega = 13.51; //in rad/s

float dc_motor_voltage = 12;

float neverest_k_i = dc_motor_voltage/neverest_max_omega;
float neverest_k_t_over_R = neverest_max_torque/dc_motor_voltage;

#define continuous_servo_stop 0.5

//globals
float dt;
float current_time;

double *ptime;
#define time (*ptime)

int * pcurrent_color;
#define current_color (*pcurrent_color)


#define min_motor_power 0.05
#define debuzz(a) (a)//(((a) < min_motor_power && (a) > -min_motor_power) ? 0 : (a))

//TODO: find shorter but descriptive name
void lowpassFirstDerivativeUpdate(float new_theta, float * current_theta, float * omega, float decay_constant)
{
    *omega = lerp((new_theta-(*current_theta))/dt, (*omega), exp(-decay_constant*dt));
    *current_theta = new_theta;
}

#define deadzone_radius 0.125

float deadzoneAdjust(float a)
{
    if (a > deadzone_radius) return (a - deadzone_radius) / (1 - deadzone_radius);
    if (a < -deadzone_radius) return (a + deadzone_radius) / (1 - deadzone_radius);
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
        stick * ((stick_norm - deadzone_radius) / (1.0f - deadzone_radius)) / stick_norm;
    } //Remap non-deadzone to full range. Unnecessary if we can't move at 10% pwm
    return stick;
}

struct smoothed_joystick
{
    union
    {
        v2f stick;
        struct
        {
            float x;
            float y;
        };
    };
    bool dead;
};

smoothed_joystick deadzoneWDead(v2f stick)
{
    smoothed_joystick out = {stick, false};
    auto stick_norm = norm(out.stick);
    if (stick_norm < deadzone_radius)
    {
        out.dead = true;
        out.x = 0;
        out.y = 0;
    }
    else
    {
        out.dead = false;
        out.stick * ((stick_norm - deadzone_radius) / (1.0f - deadzone_radius)) / stick_norm;
    } //Remap non-deadzone to full range. Unnecessary if we can't move at 10% pwm
    return out;
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
