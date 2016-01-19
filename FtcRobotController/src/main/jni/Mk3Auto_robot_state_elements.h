/*
WARNING: this is a generated file
changes made this file are not permanent
*/

#include "jni_functions.h"

enum robot_state_element
{
    rsid_left_drive_encoder, rsid_left_drive_encoder_end = rsid_left_drive_encoder + 3,
    rsid_shoulder_encoder, rsid_shoulder_encoder_end = rsid_shoulder_encoder + 3,
    rsid_heading, rsid_heading_end = rsid_heading + 3,
    rsid_roll, rsid_roll_end = rsid_roll + 3,
    rsid_y_velocity, rsid_y_velocity_end = rsid_y_velocity + 3,
    rsid_left_drive, rsid_left_drive_end = rsid_left_drive + 3,
    rsid_elbow, rsid_elbow_end = rsid_elbow + 3,
    rsid_intake, rsid_intake_end = rsid_intake + 3,
    rsid_slide, rsid_slide_end = rsid_slide + 3,
    rsid_indicator, rsid_indicator_end = rsid_indicator + 3,
    rsid_size
};

#define left_drive_encoder (*((int *) (robot_state.state+rsid_left_drive_encoder)))
#define shoulder_encoder (*((int *) (robot_state.state+rsid_shoulder_encoder)))
#define heading (*((float *) (robot_state.state+rsid_heading)))
#define roll (*((float *) (robot_state.state+rsid_roll)))
#define y_velocity (*((float *) (robot_state.state+rsid_y_velocity)))
#define left_drive (*((float *) (robot_state.state+rsid_left_drive)))
#define elbow (*((float *) (robot_state.state+rsid_elbow)))
#define intake (*((float *) (robot_state.state+rsid_intake)))
#define slide (*((float *) (robot_state.state+rsid_slide)))
#define indicator (*((int *) (robot_state.state+rsid_indicator)))
