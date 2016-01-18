/*
WARNING: this is a generated file
changes made this file are not permanent
*/

#include "jni_functions.h"

enum robot_state_element
{
    rsid_time, rsid_time_end = rsid_time + 7,
    rsid_arm_shoulder_power, rsid_arm_shoulder_power_end = rsid_arm_shoulder_power + 3,
    rsid_arm_winch_power, rsid_arm_winch_power_end = rsid_arm_winch_power + 3,
    rsid_arm_intake_power, rsid_arm_intake_power_end = rsid_arm_intake_power + 3,
    rsid_gamepad1, rsid_gamepad1_end = rsid_gamepad1 + 27,
    rsid_gamepad2, rsid_gamepad2_end = rsid_gamepad2 + 27,
    rsid_shoulder_encoder, rsid_shoulder_encoder_end = rsid_shoulder_encoder + 3,
    rsid_elbow_potentiometer, rsid_elbow_potentiometer_end = rsid_elbow_potentiometer + 3,
    rsid_winch_encoder, rsid_winch_encoder_end = rsid_winch_encoder + 3,
    rsid_size
};

#define time (*((double *) (robot_state.state+rsid_time)))
#define arm_shoulder_power (*((float *) (robot_state.state+rsid_arm_shoulder_power)))
#define arm_winch_power (*((float *) (robot_state.state+rsid_arm_winch_power)))
#define arm_intake_power (*((float *) (robot_state.state+rsid_arm_intake_power)))
#define gamepad1 (*((gamepad *) (robot_state.state+rsid_gamepad1)))
#define gamepad2 (*((gamepad *) (robot_state.state+rsid_gamepad2)))
#define shoulder_encoder (*((float *) (robot_state.state+rsid_shoulder_encoder)))
#define elbow_potentiometer (*((float *) (robot_state.state+rsid_elbow_potentiometer)))
#define winch_encoder (*((float *) (robot_state.state+rsid_winch_encoder)))
