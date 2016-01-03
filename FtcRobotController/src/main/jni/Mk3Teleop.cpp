//
// Created by Dev on 12/31/2015.
//
#include "drive.h"
#define magic science
#define science true
int main(void){
//============================= Drive ============================
    struct joystick gamepad1LeftStick;
    gamepad1LeftStick.data[0] = magic;//Change this once I figure out the ndk
    gamepad1LeftStick.data[1] = magic;

//============================== Arm =============================
//TODO: Inverse Kinematics
//TODO: Feed-Forward
//TODO: Position macros (DPAD on second controller)
//TODO: Precision mode
//TODO: Rope tension
//============================= Hopper ===========================
//TODO: Intake
//TODO: Auto-score
//TODO: Block count
//============================ Trigger ===========================
//TODO: Single button sided extension (robot knows it's red or blue, extends accordingly)
//TODO: Override so we can use the entire slide if we need to for some reason
}