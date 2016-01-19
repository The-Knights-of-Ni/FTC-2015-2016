/*
  robot_state_elements
  {
  gamepad{float joystick1_x, float joystick1_y, float joystick2_x, float joystick2_y, float left_trigger, float right_trigger, int buttons};  
  float left_drive_power;
  float right_drive_power;
  gamepad gamepad1;
  byte[100] array_example;

  int left_bumper;
  int right_bumper;
  
  int toggle_left_bumper;
  int toggle_right_bumper;
  
  //general syntax
  //type{primitive_type name in java, ...};
  //type name;
  }
*/

#include "misc.h"
#include "maths.h"
#include "Button.h"

#include "test_robot_state_elements.h"

#define JNI_main Java_com_qualcomm_ftcrobotcontroller_opmodes_NDK_1test_main

extern "C"
void JNI_main(JNIEnv * _env, jobject _self)
{
    env = _env;
    self = _self;
    
    initJNI();
    
    Button pad1;
    Button pad2;
    
    waitForStart();
    float time = 0;
    for ever
    {
        //(*((float*)(robot_state.state+rsid_left_drive_power))) = 1.0f;//robot_state.state[rsid_gamepad1];
        left_drive_power = gamepad1.joystick1.y-gamepad1.joystick1.x;
        right_drive_power = gamepad1.joystick1.y+gamepad1.joystick1.x;
        
        toggle_left_bumper = pad1.toggle(LEFT_BUMPER);
        toggle_right_bumper = pad1.toggle(A);
        left_bumper = pad1.press(LEFT_BUMPER);
        right_bumper = pad1.press(A);
        
        pad1.updateButtons(gamepad1.buttons);
        //pad2.updateButtons(gamepad2.buttons);
        updateRobot();
    }
}
