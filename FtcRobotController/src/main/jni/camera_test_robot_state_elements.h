/*
WARNING: this is a generated file
changes made this file are not permanent
*/

#include "jni_functions.h"

enum robot_state_element
{
    rsid_camera_w, rsid_camera_w_end = rsid_camera_w + 3,
    rsid_camera_h, rsid_camera_h_end = rsid_camera_h + 3,
    rsid_size
};

#define camera_w (*((int *) (robot_state.state+rsid_camera_w)))
#define camera_h (*((int *) (robot_state.state+rsid_camera_h)))
