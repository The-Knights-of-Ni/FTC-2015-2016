/*
WARNING: this is a generated file
changes made this file are not permanent
*/

#include "jni_functions.h"

enum robot_state_element
{
    rsid_left_drive_power, rsid_left_drive_power_end = rsid_left_drive_power + 3,
    rsid_right_drive_power, rsid_right_drive_power_end = rsid_right_drive_power + 3,
    rsid_gamepad1, rsid_gamepad1_end = rsid_gamepad1 + 27,
    rsid_array_example, rsid_array_example_end = rsid_array_example + 99,
    rsid_size
};

#define left_drive_power (*((float *) (robot_state.state+rsid_left_drive_power)))
#define right_drive_power (*((float *) (robot_state.state+rsid_right_drive_power)))
#define gamepad1 (*((gamepad *) (robot_state.state+rsid_gamepad1)))
#define array_example (((byte *) (robot_state.state+rsid_array_example)))
