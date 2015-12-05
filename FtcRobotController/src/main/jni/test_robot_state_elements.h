/*
WARNING: this is a generated file
changes made this file are not permanent
*/
#include "jni_functions.h"
enum robot_state_element
{
    left_drive_power, left_drive_power_end = left_drive_power + 4,
    right_drive_power, right_drive_power_end = right_drive_power + 4,
    gamepad1, gamepad1_end = gamepad1 + 8,
    array_example, array_example_end = array_example + 100,
};

float & left_drive_power = ((float *) robot_state.state)[rsid_left_drive_power];
float & right_drive_power = ((float *) robot_state.state)[rsid_right_drive_power];
gamepad & gamepad1 = ((gamepad *) robot_state.state)[rsid_gamepad1];
byte & array_example = ((byte *) robot_state.state)[rsid_array_example];
