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
    rsid_imu_heading, rsid_imu_heading_end = rsid_imu_heading + 3,
    rsid_imu_tilt, rsid_imu_tilt_end = rsid_imu_tilt + 3,
    rsid_imu_roll, rsid_imu_roll_end = rsid_imu_roll + 3,
    rsid_imu_velocity, rsid_imu_velocity_end = rsid_imu_velocity + 11,
    rsid_color, rsid_color_end = rsid_color + 3,
    rsid_left_drive, rsid_left_drive_end = rsid_left_drive + 3,
    rsid_right_drive, rsid_right_drive_end = rsid_right_drive + 3,
    rsid_winch, rsid_winch_end = rsid_winch + 3,
    rsid_shoulder, rsid_shoulder_end = rsid_shoulder + 3,
    rsid_intake, rsid_intake_end = rsid_intake + 3,
    rsid_hand, rsid_hand_end = rsid_hand + 3,
    rsid_slide, rsid_slide_end = rsid_slide + 3,
    rsid_indicator, rsid_indicator_end = rsid_indicator + 3,
    rsid_camera_w, rsid_camera_w_end = rsid_camera_w + 3,
    rsid_camera_h, rsid_camera_h_end = rsid_camera_h + 3,
    rsid_beacon_right, rsid_beacon_right_end = rsid_beacon_right + 3,
    rsid_size
};

#define time (*((double *) (robot_state.state+rsid_time)))
#define right_drive_encoder (*((int *) (robot_state.state+rsid_right_drive_encoder)))
#define left_drive_encoder (*((int *) (robot_state.state+rsid_left_drive_encoder)))
#define winch_encoder (*((int *) (robot_state.state+rsid_winch_encoder)))
#define shoulder_encoder (*((int *) (robot_state.state+rsid_shoulder_encoder)))
#define elbow_potentiometer (*((int *) (robot_state.state+rsid_elbow_potentiometer)))
#define imu_heading (*((float *) (robot_state.state+rsid_imu_heading)))
#define imu_tilt (*((float *) (robot_state.state+rsid_imu_tilt)))
#define imu_roll (*((float *) (robot_state.state+rsid_imu_roll)))
#define imu_velocity (*((v3f *) (robot_state.state+rsid_imu_velocity)))
#define color (*((int *) (robot_state.state+rsid_color)))
#define left_drive (*((float *) (robot_state.state+rsid_left_drive)))
#define right_drive (*((float *) (robot_state.state+rsid_right_drive)))
#define winch (*((float *) (robot_state.state+rsid_winch)))
#define shoulder (*((float *) (robot_state.state+rsid_shoulder)))
#define intake (*((float *) (robot_state.state+rsid_intake)))
#define hand (*((float *) (robot_state.state+rsid_hand)))
#define slide (*((float *) (robot_state.state+rsid_slide)))
#define indicator (*((int *) (robot_state.state+rsid_indicator)))
#define camera_w (*((int *) (robot_state.state+rsid_camera_w)))
#define camera_h (*((int *) (robot_state.state+rsid_camera_h)))
#define beacon_right (*((int *) (robot_state.state+rsid_beacon_right)))
