//This is not a comment
/*
  robot_state_elements
  {
  gamepad{float joystick1_x, float joystick1_y, float joystick2_x, float joystick2_y, float trigger1, float trigger2, int buttons};

  float left_drive;
  float right_drive;
  float elbow;
  float shoulder;
  float intake;
  float hand;
  float slide;

  gamepad gamepad1;
  gamepad gamepad2;

  //general syntax
  //type{primitive_type name in java, ...};
  //type name;
  }
*/

#include "drive.h"
#include "arm.h"
#include "Button.h"

//TODO: generate this
#define JNI_main Java_com_qualcomm_ftcrobotcontroller_opmodes_MK3Teleop_main
extern "C"


//TODO: Get RED/BLUE Status
#define current_color 0 //0 = red, 1 = blue

#define slide_rotations 20 //TODO: Move this to <robotname>.h
#define hand_blue_position 1
#define hand_red_position -1
#define hand_level_position 0
#define slide_speed 5
#define slide_blue_position 1
#define slide_red_position -1
#define slide_stored_position 0
//KEYBINDS
#define drive_stick pad1stick1


//Hopper
#define intake_toggle pad1.toggle(Buttons::LEFT_BUMPER)
#define intake_reverse pad1.press(Buttons::RIGHT_BUMPER)
#define hopper_tilt pad1.toggle(Buttons::A) //Might make this a stick or something
//Arm
#define shoulder_manual pad2stick1
#define elbow_manual pad2stick2
#define arm_stick pad2stick1
#define arm_manual_toggle pad2.toggle(Buttons::LEFT_STICK_BUTTON)
//Slide
#define slide_toggle pad1.toggle(Buttons::Y)
#define slide_right pad1.press(Buttons::DPAD_RIGHT)
#define slide_left pad1.press(Buttons::DPAD_LEFT)

void JNI_main(JNIEnv * env, jobject self)
{
    initJNI(env, self);
    Button pad1;
    Button pad2;
    v2f pad1stick1;
    v2f pad1stick2;
    v2f pad2stick1;
    v2f pad2stick2;
    for ever
    {
        pad1stick1.x = gamepad1.joystick1_x; pad1stick1.y = gamepad1.joystick1_y;
        pad1stick2.x = gamepad1.joystick2_x; pad1stick2.y = gamepad1.joystick2_y;
        pad2stick1.x = gamepad2.joystick1_x; pad2stick1.y = gamepad2.joystick1_y;
        pad2stick2.x = gamepad2.joystick2_x; pad2stick2.y = gamepad2.joystick2_y;
//============================= Drive ============================
        deadZone(drive_stick);
        smoothJoysticks(drive_stick);
        left_drive = drive_stick.y - drive_stick.x;
        right_drive = drive_stick.y + drive_stick.x;
        //Might need to add additional bounding in as a safety

//============================== Arm =============================
        if(!arm_manual_toggle) //IK
        {
            deadZone(arm_stick);//Also deadzoning shoulder_manual
            getArmTargetsPolar(arm_stick, arm_manual_toggle);
        }
        else //Manual
        {
            deadZone(elbow_manual);
            deadZone(shoulder_manual);
            smoothJoysticks(elbow_manual);//This needs to be fixed.
            smoothJoysticks(shoulder_manual);
            shoulder = shoulder_manual.y;
            elbow = elbow_manual.y;
        }


//TODO: Feed-Forward
//TODO: Position macros (DPAD on second controller)
//TODO: Precision mode
//TODO: Rope tension
//TODO: Locks
//============================= Hopper ===========================
        if(intake_toggle)
            intake = intake_reverse ? -1 : 1;
        if(hopper_tilt)
        {
            if(current_color)
                hand = hand_blue_position;
            else
                hand = hand_red_position;
        }
        else
            hand = hand_level_position;
//TODO: Auto-score
//TODO: Block count
        //TODO: Tilt
//============================ Slide ===========================
        if(slide_toggle)
        {
            if(current_color)
                slide = slide_blue_position;
            else
                slide = slide_red_position;
        }
        else
            slide = slide_stored_position;

        //These aren't working yet, I'll need to write a release condition (the toggle is forcing it closed)
        if(slide_right)
            slide += slide_speed;
        if(slide_left)
            slide -= slide_speed;
//============================ Updates ===========================
        gamepad1.updateButtons(gamepad1.buttons);
        gamepad2.updateButtons(gamepad2.buttons);
        updateRobot(env, self);
    }

    cleanupJNI(env, self);
}
