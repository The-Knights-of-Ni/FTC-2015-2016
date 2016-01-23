/*
WARNING: this is a generated file
changes made this file are not permanent
*/

#include "jni_functions.h"

enum robot_state_element
{
    rsid_time, rsid_time_end = rsid_time + 7,
    rsid_right_drive_encoder, rsid_right_drive_encoder_end = rsid_right_drive_encoder + 3,
    rsid_left_drive_encoder, rsid_left_drive_encoder_end = rsid_left_drive_encoder + 3,
    rsid_winch_encoder, rsid_winch_encoder_end = rsid_winch_encoder + 3,
    rsid_shoulder_encoder, rsid_shoulder_encoder_end = rsid_shoulder_encoder + 3,
    rsid_elbow_potentiometer, rsid_elbow_potentiometer_end = rsid_elbow_potentiometer + 3,
    rsid_heading, rsid_heading_end = rsid_heading + 3,
    rsid_tilt, rsid_tilt_end = rsid_tilt + 3,
    rsid_roll, rsid_roll_end = rsid_roll + 3,
    rsid_x_velocity, rsid_x_velocity_end = rsid_x_velocity + 3,
    rsid_y_velocity, rsid_y_velocity_end = rsid_y_velocity + 3,
    rsid_current_color, rsid_current_color_end = rsid_current_color + 3,
    rsid_left_drive, rsid_left_drive_end = rsid_left_drive + 3,
    rsid_right_drive, rsid_right_drive_end = rsid_right_drive + 3,
    rsid_winch, rsid_winch_end = rsid_winch + 3,
    rsid_shoulder, rsid_shoulder_end = rsid_shoulder + 3,
    rsid_intake, rsid_intake_end = rsid_intake + 3,
    rsid_hand, rsid_hand_end = rsid_hand + 3,
    rsid_slide, rsid_slide_end = rsid_slide + 3,
    rsid_hook, rsid_hook_end = rsid_hook + 3,
    rsid_shoulder_print_theta, rsid_shoulder_print_theta_end = rsid_shoulder_print_theta + 3,
    rsid_forearm_print_theta, rsid_forearm_print_theta_end = rsid_forearm_print_theta + 3,
    rsid_gamepad1, rsid_gamepad1_end = rsid_gamepad1 + 27,
    rsid_gamepad2, rsid_gamepad2_end = rsid_gamepad2 + 27,
    rsid_size
};

#define time (*((double *) (robot_state.state+rsid_time)))
#define right_drive_encoder (*((int *) (robot_state.state+rsid_right_drive_encoder)))
#define left_drive_encoder (*((int *) (robot_state.state+rsid_left_drive_encoder)))
#define winch_encoder (*((int *) (robot_state.state+rsid_winch_encoder)))
#define shoulder_encoder (*((int *) (robot_state.state+rsid_shoulder_encoder)))
#define elbow_potentiometer (*((int *) (robot_state.state+rsid_elbow_potentiometer)))
#define heading (*((float *) (robot_state.state+rsid_heading)))
#define tilt (*((float *) (robot_state.state+rsid_tilt)))
#define roll (*((float *) (robot_state.state+rsid_roll)))
#define x_velocity (*((float *) (robot_state.state+rsid_x_velocity)))
#define y_velocity (*((float *) (robot_state.state+rsid_y_velocity)))
#define current_color (*((int *) (robot_state.state+rsid_current_color)))
#define left_drive (*((float *) (robot_state.state+rsid_left_drive)))
#define right_drive (*((float *) (robot_state.state+rsid_right_drive)))
#define winch (*((float *) (robot_state.state+rsid_winch)))
#define shoulder (*((float *) (robot_state.state+rsid_shoulder)))
#define intake (*((float *) (robot_state.state+rsid_intake)))
#define hand (*((float *) (robot_state.state+rsid_hand)))
#define slide (*((float *) (robot_state.state+rsid_slide)))
#define hook (*((float *) (robot_state.state+rsid_hook)))
#define shoulder_print_theta (*((float *) (robot_state.state+rsid_shoulder_print_theta)))
#define forearm_print_theta (*((float *) (robot_state.state+rsid_forearm_print_theta)))
#define gamepad1 (*((gamepad *) (robot_state.state+rsid_gamepad1)))
#define gamepad2 (*((gamepad *) (robot_state.state+rsid_gamepad2)))
