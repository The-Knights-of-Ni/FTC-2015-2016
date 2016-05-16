#ifndef WHITE_RABBIT
#define WHITE_RABBIT

#include "drive.h"
#include "Button.h"
#ifndef USING_SIMULATOR
#include "arm.h"

#include "spline.h"
#endif
//Deadzones, bounds, and smooths the joysticks completely.
smoothed_joystick smoothJoysticks(v2f stick, float min_power, float x_mod, float y_mod, float max_power)
{
    smoothed_joystick out = deadzoneWDead(stick);
    out.x = clamp(out.x, -1, 1);
    out.y = clamp(out.y, -1, 1);
    out.x = (out.x < 0 ? -1 : 1) * quadraticBezier(out.x, min_power, x_mod, max_power);
    out.y = (out.y < 0 ? -1 : 1) * quadraticBezier(out.y, min_power, y_mod, max_power);
    return out;
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
