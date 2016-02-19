#ifndef WHITE_RABBIT
#define WHITE_RABBIT

#include "drive.h"
#include "arm.h"
#include "Button.h"

//Deadzones, bounds, and smooths the joysticks completely.
v2f smoothJoysticks(v2f stick, float min_power, float x_mod, float y_mod, float max_power)
{
    stick = deadzone(stick);
    stick.x = clamp(stick.x, -1, 1);
    stick.y = clamp(stick.y, -1, 1);
    stick.x = (stick.x < 0 ? -1 : 1) * quadraticBezier(stick.x, min_power, x_mod, max_power);
    stick.y = (stick.y < 0 ? -1 : 1) * quadraticBezier(stick.y, min_power, y_mod, max_power);
    return stick;
}
//Deadzones, bounds, and smooths an axis completely.
float smoothAxis(float axis, float min_power, float mod, float max_power)
{
    axis = deadzoneAdjust(axis);
    axis = clamp(axis, -1, 1);
    axis = (axis < 0 ? -1 : 1) * quadraticBezier(axis, min_power, mod, max_power);
    return axis;
}

#endif //WHITE_RABBIT
