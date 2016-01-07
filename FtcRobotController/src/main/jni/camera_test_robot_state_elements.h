/*
WARNING: this is a generated file
changes made this file are not permanent
*/

#include "jni_functions.h"

enum robot_state_element
{
    rsid_camera_buffer, rsid_camera_buffer_end = rsid_camera_buffer + 76799,
    rsid_overlay_buffer, rsid_overlay_buffer_end = rsid_overlay_buffer + 76799,
    rsid_size
};

#define camera_buffer (((byte *) (robot_state.state+rsid_camera_buffer)))
#define overlay_buffer (((byte *) (robot_state.state+rsid_overlay_buffer)))
