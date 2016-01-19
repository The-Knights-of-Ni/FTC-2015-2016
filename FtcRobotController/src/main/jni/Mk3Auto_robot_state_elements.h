/*
WARNING: this is a generated file
changes made this file are not permanent
*/

#include "jni_functions.h"

enum robot_state_element
{
    rsid_right_drive_encoder, rsid_right_drive_encoder_end = rsid_right_drive_encoder + 3,
    rsid_left_drive_encoder, rsid_left_drive_encoder_end = rsid_left_drive_encoder + 3,
    rsid_elbow_encoder, rsid_elbow_encoder_end = rsid_elbow_encoder + 3,
    rsid_shoulder_encoder, rsid_shoulder_encoder_end = rsid_shoulder_encoder + 3,
    rsid_potentiometer, rsid_potentiometer_end = rsid_potentiometer + 3,
    rsid_heading, rsid_heading_end = rsid_heading + 3,
    rsid_tilt, rsid_tilt_end = rsid_tilt + 3,
    rsid_roll, rsid_roll_end = rsid_roll + 3,
    rsid_x_velocity, rsid_x_velocity_end = rsid_x_velocity + 3,
    rsid_y_velocity, rsid_y_velocity_end = rsid_y_velocity + 3,
    rsid_left_drive, rsid_left_drive_end = rsid_left_drive + 3,
    rsid_right_drive, rsid_right_drive_end = rsid_right_drive + 3,
    rsid_elbow, rsid_elbow_end = rsid_elbow + 3,
    rsid_shoulder, rsid_shoulder_end = rsid_shoulder + 3,
    rsid_intake, rsid_intake_end = rsid_intake + 3,
    rsid_hand, rsid_hand_end = rsid_hand + 3,
    rsid_slide, rsid_slide_end = rsid_slide + 3,
    rsid_indicator, rsid_indicator_end = rsid_indicator + 3,
    rsid_size
};

#define right_drive_encoder (*((int *) (robot_state.state+rsid_right_drive_encoder)))
#define left_drive_encoder (*((int *) (robot_state.state+rsid_left_drive_encoder)))
#define elbow_encoder (*((int *) (robot_state.state+rsid_elbow_encoder)))
#define shoulder_encoder (*((int *) (robot_state.state+rsid_shoulder_encoder)))
#define potentiometer (*((int *) (robot_state.state+rsid_potentiometer)))
#define heading (*((float *) (robot_state.state+rsid_heading)))
#define tilt (*((float *) (robot_state.state+rsid_tilt)))
#define roll (*((float *) (robot_state.state+rsid_roll)))
#define x_velocity (*((float *) (robot_state.state+rsid_x_velocity)))
#define y_velocity (*((float *) (robot_state.state+rsid_y_velocity)))
#define left_drive (*((float *) (robot_state.state+rsid_left_drive)))
#define right_drive (*((float *) (robot_state.state+rsid_right_drive)))
#define elbow (*((float *) (robot_state.state+rsid_elbow)))
#define shoulder (*((float *) (robot_state.state+rsid_shoulder)))
#define intake (*((float *) (robot_state.state+rsid_intake)))
#define hand (*((float *) (robot_state.state+rsid_hand)))
#define slide (*((float *) (robot_state.state+rsid_slide)))
#define indicator (*((int *) (robot_state.state+rsid_indicator)))
