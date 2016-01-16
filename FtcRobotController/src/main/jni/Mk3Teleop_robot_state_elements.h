/*
WARNING: this is a generated file
changes made this file are not permanent
*/

#include "jni_functions.h"

enum robot_state_element
{
    rsid_left_drive, rsid_left_drive_end = rsid_left_drive + 3,
    rsid_right_drive, rsid_right_drive_end = rsid_right_drive + 3,
    rsid_elbow, rsid_elbow_end = rsid_elbow + 3,
    rsid_shoulder, rsid_shoulder_end = rsid_shoulder + 3,
    rsid_intake, rsid_intake_end = rsid_intake + 3,
    rsid_hand, rsid_hand_end = rsid_hand + 3,
    rsid_slide, rsid_slide_end = rsid_slide + 3,
    rsid_gamepad1, rsid_gamepad1_end = rsid_gamepad1 + 27,
    rsid_gamepad2, rsid_gamepad2_end = rsid_gamepad2 + 27,
    rsid_size
};

#define left_drive (*((float *) (robot_state.state+rsid_left_drive)))
#define right_drive (*((float *) (robot_state.state+rsid_right_drive)))
#define elbow (*((float *) (robot_state.state+rsid_elbow)))
#define shoulder (*((float *) (robot_state.state+rsid_shoulder)))
#define intake (*((float *) (robot_state.state+rsid_intake)))
#define hand (*((float *) (robot_state.state+rsid_hand)))
#define slide (*((float *) (robot_state.state+rsid_slide)))
#define gamepad1 (*((gamepad *) (robot_state.state+rsid_gamepad1)))
#define gamepad2 (*((gamepad *) (robot_state.state+rsid_gamepad2)))
