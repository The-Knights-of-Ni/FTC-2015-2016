/*
WARNING: this is a generated file
changes made this file are not permanent
*/
enum robot_state_element
{
    rsid_left_drive_power, rsid_left_drive_power_end = rsid_left_drive_power + 4,
    rsid_right_drive_power, rsid_right_drive_power_end = rsid_right_drive_power + 4,
    rsid_gamepad1, rsid_gamepad1_end = rsid_gamepad1 + 8,
    rsid_array_example, rsid_array_example_end = rsid_array_example + 100,
    rsid_size
};

#include "jni_functions.h"


float & left_drive_power = ((float *) robot_state.state)[rsid_left_drive_power];
float & right_drive_power = ((float *) robot_state.state)[rsid_right_drive_power];
gamepad & gamepad1 = ((gamepad *) robot_state.state)[rsid_gamepad1];
byte & array_example = ((byte *) robot_state.state)[rsid_array_example];
