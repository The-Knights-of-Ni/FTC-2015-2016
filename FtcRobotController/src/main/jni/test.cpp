/*
  robot_state_elements
  {
  gamepad{float joystick_1_x, float joystick_2_y};
  
  float left_drive_power;
  float right_drive_power;
  gamepad gamepad1;
  byte[100] array_example;
  
  //general syntax
  //type{primitive_type name in java, ...};
  //type name;
  }
*/

#include "misc.h"
#include "maths.h"

#include <jni.h>

// #define reserveType(type, name) name, name##end = name + sizeof(type)
// #define reserveBytes(n_bytes, name) name, name##size = (n_bytes), name##end = name + (n_bytes)-1
// #define reserveArray(type, n_elements, name) name, name##size = sizeof(type)*(n_bytes), name##end = name + sizeof(type)*(n_bytes)-1

struct gamepad
{
    v2f left_stick;
};
#include "test_robot_state_elements.h"

// enum robot_state_element
// {
//     reserveType(float, rsid_left_drive_power),
//     reserveType(float, rsid_right_drive_power),
//     reserveType(gamepad, rsid_gamepad1),
//     //reserveBytes(100, rsid_array_example),
//     rsid_size,
// };

// #undef reserve_int
// #undef reserve_float

// #define add_robot_state_element(type, name) type & name = ((type *) robot_state.state)[rsid_##name]
// add_robot_state_element(float, left_drive_power);
// add_robot_state_element(float, right_drive_power);
    
// add_robot_state_element(gamepad, gamepad1);

//add_robot_state_element(char, array_example);

// #undef add_robot_state_element

extern "C"
void JNI_main(JNIEnv * env, jobject self)
{
    initJNI(env, self);
    
    waitForStart();
    float time = 0;
    for ever
    {
        left_drive_power = gamepad1.left_stick.y-gamepad1.left_stick.x;
        right_drive_power = gamepad1.left_stick.y+gamepad1.left_stick.x;
        updateRobot(env, self);
    }
    cleanupJNI(env, self);
}
