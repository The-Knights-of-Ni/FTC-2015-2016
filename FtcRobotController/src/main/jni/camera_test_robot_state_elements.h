/*
WARNING: this is a generated file
changes made this file are not permanent
*/

#include "jni_functions.h"

enum robot_state_element
{
    rsid_camera_buffers, rsid_camera_buffers_end = rsid_camera_buffers + 9830399,
    rsid_current_buffer, rsid_current_buffer_end = rsid_current_buffer + 3,
    rsid_size
};

#define camera_buffers (((byte *) (robot_state.state+rsid_camera_buffers)))
#define current_buffer (*((int *) (robot_state.state+rsid_current_buffer)))
