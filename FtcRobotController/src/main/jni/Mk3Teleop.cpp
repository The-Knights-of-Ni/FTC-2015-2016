#include "drive.h"
#include "arm.h"
#include "robotics.h"
#include "Button.h"

#include "jni_functions.h"

//TODO: generate this
#ifndef GENERATE
#undef jniMain
#define jniMain Java_com_qualcomm_ftcrobotcontroller_opmodes_Mk3Teleop_main
#endif

//TODO: Get RED/BLUE Status
// #define current_color 0 //0 = red, 1 = blue

#define slide_rotations 20 //TODO: Move this to <robotname>.h
float hand_red_position = 0.9;
float hand_blue_position = 0.3;
float hand_level_position = 0.6;
#define slide_speed 5
#define slide_blue_position 1
#define slide_red_position -1
#define slide_stored_position 0

float hook_level_position = 0.0f;
float hook_locked_position = 1.0f;
//KEYBINDS

//Drive
#define drive_stick pad1stick1
#define drive_toggle pad1.toggle(LEFT_STICK_BUTTON)

//Hopper
#define intake_toggle pad1.toggle(LEFT_BUMPER)
#define intake_reverse pad1.press(RIGHT_BUMPER)
#define hopper_tilt pad2.toggle(RIGHT_BUMPER) //Might make this a stick or something
#define hand_manual_control pad1stick2.x

//Arm
// #define shoulder_manual pad2stick1
// #define elbow_manual pad2stick2
//                       [0] shoulder, [1] winch/elbow
#define arm_stick ((v2f){pad2stick1.y, -pad2stick2.y})
#define arm_manual_toggle pad2.toggle(Y)
#define arm_score_mode_button pad2.press(LEFT_TRIGGER)
#define arm_intake_mode_button pad2.press(LEFT_BUMPER)
#define precision_mode pad2.toggle(A)

//Slide
//#define slide_toggle pad1.toggle(Y)
#define slide_right pad1.press(DPAD_RIGHT)
#define slide_left pad1.press(DPAD_LEFT)

//Hook
#define hook_toggle pad1.toggle(B)
extern "C"
void jniMain(JNIEnv * _env, jobject _self)
{
    env = _env;
    self = _self;
    initJNI();
    
    jni_import_string = ("import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;\n"
                         "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n"
                         "import com.qualcomm.robotcore.exception.RobotCoreException;\n"
                         "import com.qualcomm.robotcore.hardware.DcMotor;\n"
                         "import com.qualcomm.robotcore.hardware.DcMotorController;\n"
                         "import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;\n"
                         "import com.qualcomm.robotcore.hardware.Servo;\n");
    
    //TODO: shortcut for defining and declaring motors, servos, etc.
    jni_variables_string = ("/* Start Motor Definitions */\n"
                            "int elbow_potentiometer_port = 7;\n"
                            "int shoulder_potentiometer_port = 1;\n"
                            "DeviceInterfaceModule dim;\n"
                            "\n"
                            "DcMotor left_drive;\n"
                            "DcMotor right_drive;\n"
                            "DcMotor shoulder;\n"
                            "DcMotor winch;\n"
                            "DcMotor intake;\n"
                            "\n"
                            "Servo hand;\n"
                            "Servo slide;\n"
                            "Servo hook_left;\n"
                            "Servo hook_right;\n"
                            "/* End Motor Definitions */");
    
    jni_run_opmode_string = ("dim = hardwareMap.deviceInterfaceModule.get(\"dim\");\n"
                             "//left_drive  = hardwareMap.dcMotor.get(\"leftd\");\n"
                             "//right_drive = hardwareMap.dcMotor.get(\"rightd\");\n"
                             "shoulder    = hardwareMap.dcMotor.get(\"shoulder\");\n"
                             "winch       = hardwareMap.dcMotor.get(\"winch\");\n"
                             "//intake      = hardwareMap.dcMotor.get(\"intake\");\n"
                             "//right_drive.setDirection(DcMotor.Direction.REVERSE);\n"
                             "//left_drive.setDirection(DcMotor.Direction.REVERSE);\n"
                             "shoulder.setDirection(DcMotor.Direction.REVERSE);\n"
                             "shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);\n"
                             "waitOneFullHardwareCycle();\n"
                             "//intake.setDirection(DcMotor.Direction.REVERSE);\n"
                             "shoulder.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);\n"
                             "winch.setMode(DcMotorController.RunMode.RESET_ENCODERS);\n"
                             "waitOneFullHardwareCycle();\n"
                             "winch.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);\n"
                             "waitOneFullHardwareCycle();\n"
                             "shoulder.setDirection(DcMotor.Direction.REVERSE);\n"
                             "//winch.setDirection(DcMotor.Direction.REVERSE);\n"
                             "\n"
                             "//hand = hardwareMap.servo.get(\"hand\");\n"
                             "//slide = hardwareMap.servo.get(\"slide\");\n"
                             "//hook_left = hardwareMap.servo.get(\"hook_left\");\n"
                             "//hook_right = hardwareMap.servo.get(\"hook_right\");\n"
                             "//hook_left.setDirection(Servo.Direction.REVERSE);");
    
    jni_misc_string = (
        "public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.\n"
        "{\n"
        "    return ByteBuffer.wrap(joystick, 42, 4).getInt();\n"
        "}\n");
    
    ptime = jniDoubleIn("return time;");
    pright_drive_encoder = jniIntIn("return 0;//right_drive.getCurrentPosition();");
    pleft_drive_encoder = jniIntIn("return 0;//left_drive.getCurrentPosition();");
    pwinch_encoder = jniIntIn("return winch.getCurrentPosition();");
    pshoulder_encoder = jniIntIn("return shoulder.getCurrentPosition();");
    pelbow_potentiometer = jniIntIn("return dim.getAnalogInputValue(elbow_potentiometer_port);");
    pshoulder_potentiometer = jniIntIn("return dim.getAnalogInputValue(shoulder_potentiometer_port);");
    int * current_color = jniIntIn("return (FtcRobotControllerActivity.red ? 1 : 0);");
    
    pslider0 = jniIntIn("return FtcRobotControllerActivity.slider_0;");
    pslider1 = jniIntIn("return FtcRobotControllerActivity.slider_1;");
    pslider2 = jniIntIn("return FtcRobotControllerActivity.slider_2;");
    pslider3 = jniIntIn("return FtcRobotControllerActivity.slider_3;");
    
    jniOut("//left_drive.setPower(", pleft_drive, ");;");
    jniOut("//right_drive.setPower(", pright_drive, ");;");
    jniOut("winch.setPower(", pwinch, ");;");
    jniOut("shoulder.setPower(", pshoulder, ");;");
    jniOut("//intake.setPower(", pintake, ");;");
    
    jniOut("//hand.setPosition(", phand,");");
    jniOut("//slide.setPosition(", pslide,");");
    jniOut("//hook_left.setPosition(", phook_left,");");
    jniOut("//hook_right.setPosition(", phook_right,");");
    
    float * pshoulder_print_theta;
    #define shoulder_print_theta (*pshoulder_print_theta)
    jniOut("telemetry.addData(\"shoulder theta\", ", pshoulder_print_theta,");");

    float * pshoulder_compensation_print;
    #define shoulder_compensation_print (*pshoulder_compensation_print)
    jniOut("telemetry.addData(\"shoulder_compensation\", ", pshoulder_compensation_print,");");

    int * pshoulder_active_print;
    #define shoulder_active_print (*pshoulder_active_print)
    jniOut("telemetry.addData(\"shoulder_active\", ", pshoulder_active_print,");");
    
    jniOut("telemetry.addData(\"slider 0\", ", pslider0,");");
    jniOut("telemetry.addData(\"slider 1\", ", pslider1,");");
    jniOut("telemetry.addData(\"slider 2\", ", pslider2,");");
    jniOut("telemetry.addData(\"slider 3\", ", pslider3,");");
    
    float * pforearm_print_theta;
    #define forearm_print_theta (*pforearm_print_theta)
    jniOut("telemetry.addData(\"forearm theta\", ", pforearm_print_theta,");");
    
    jniOut("telemetry.addData(\"shoulder power\", ", pshoulder,");");
    
    pgamepad1 = jniStructIn(
        gamepad,
        "int gamepad1_buttons = 0;\n"
        "try\n"
        "{\n"
        "    gamepad1_buttons = updateButtons(gamepad1.toByteArray());\n"
        "}\n"
        "catch (RobotCoreException e)\n"
        "{\n"
        "    e.printStackTrace();\n"
        "}\n"
        "telemetry.addData(\"gamepad1_buttons\", String.format(\"%d\", gamepad1_buttons));\n"
        "return {gamepad1.left_stick_x, gamepad1.left_stick_y,"
        "gamepad1.right_stick_x, gamepad1.right_stick_y,"
        "gamepad1.left_trigger, gamepad1.right_trigger, gamepad1_buttons};");
    
    pgamepad2 = jniStructIn(
        gamepad,
        "int gamepad2_buttons = 0;\n"
        "try\n"
        "{\n"
        "    gamepad2_buttons = updateButtons(gamepad2.toByteArray());\n"
        "}\n"
        "catch (RobotCoreException e)\n"
        "{\n"
        "    e.printStackTrace();\n"
        "}\n"
        "return {gamepad2.left_stick_x, gamepad2.left_stick_y,"
        "gamepad2.right_stick_x, gamepad2.right_stick_y,"
        "gamepad2.left_trigger, gamepad2.right_trigger, gamepad2_buttons};");
    
    jniGenerate();
    
    Button pad1 = {};
    Button pad2 = {};
    v2f pad1stick1;
    v2f pad1stick2;
    v2f pad2stick1;
    v2f pad2stick2;
    
    target_arm_theta = pi*7/12;
    target_shoulder_theta = pi*150/180;
    target_inside_elbow_theta = pi/6;
    
    shoulder_omega = 0;
    winch_omega = 0;
    inside_elbow_omega = 0;
    
    shoulder_compensation = 0;
    
    waitForStart();
    
    interruptable for ever
    {
        dt = time - current_time;
        current_time = time;
        
        if(dt == 0)
        {
            updateRobot();
            continue;
        }
        
//============================ Controls ==========================
        
        pad1stick1.x = gamepad1.joystick1.x;
        pad1stick1.y = gamepad1.joystick1.y;
        pad1stick2.x = gamepad1.joystick2.x;
        pad1stick2.y = gamepad1.joystick2.y;
        pad2stick1.x = gamepad2.joystick1.x;
        pad2stick1.y = gamepad2.joystick1.y;
        pad2stick2.x = gamepad2.joystick2.x;
        pad2stick2.y = gamepad2.joystick2.y;
        
//============================= Drive ============================
        drive_stick = deadzone(drive_stick);
        //smoothJoysticks(&drive_stick);
        if(drive_toggle)
        {
            left_drive = -drive_stick.y - drive_stick.x;
            right_drive = -drive_stick.y + drive_stick.x;
        }
        else
        {
            left_drive = drive_stick.y - drive_stick.x;
            right_drive = drive_stick.y + drive_stick.x;
        }
        left_drive = clamp(left_drive, -1.0, 1.0);
        right_drive = clamp(right_drive, -1.0, 1.0);
        //Might need to add additional bounding in as a safety
        
//============================== Arm =============================            
        updateArmSensors();

        shoulder_compensation_print = shoulder_compensation;
        shoulder_print_theta = shoulder_theta;
        forearm_print_theta = shoulder_omega;
        shoulder_active_print = shoulder_active;
        
        if(arm_manual_toggle) //IK
        {
            v2f target_arm_velocity = arm_stick;
            if(arm_score_mode_button)
            {
                winch = 0;
                shoulder = 0;
                
                score_mode = true;
                target_shoulder_theta = 1.0;
                target_inside_elbow_theta = pi*4/5;
                
                float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
                target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
                
                shoulder = 1*(target_shoulder_theta-shoulder_theta);
                winch = (1*(target_inside_elbow_theta-inside_elbow_theta)
                         +shoulder*winch_gear_ratio/shoulder_gear_ratio);
            }
            else if(arm_intake_mode_button)
            {
                winch = 0;
                shoulder = 0;
                
                score_mode = false;
                target_shoulder_theta = 2.0;
                target_inside_elbow_theta = 3.5;
                
                shoulder = 1*(target_shoulder_theta-shoulder_theta);
                winch = (1*(target_inside_elbow_theta-inside_elbow_theta)
                         +shoulder*winch_gear_ratio/shoulder_gear_ratio);
            }
            else
            {
                if(score_mode)
                {
                    armAtVelocity(target_arm_velocity);
                    
                    target_shoulder_theta = shoulder_theta;
                }
                else
                {
                    armJointsAtVelocity(target_arm_velocity);
                    
                    float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
                    target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
                }
                
                if(score_mode)
                {
                    armToAngle();
                }
                else
                {
                    armJointsToAngle();
                }
            }
            
            shoulder = clamp(shoulder, -1.0, 1.0);
            winch = clamp(winch, -1.0, 1.0);
        }
        else //Manual
        {
            float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                              -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
            target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
            target_shoulder_theta = shoulder_theta;
            target_inside_elbow_theta = inside_elbow_theta;
            
            float shoulder_control = filterArmJoystick(arm_stick[0]);
            float elbow_control = filterArmJoystick(arm_stick[1]);
            // smoothJoysticks(&elbow_manual);//This needs to be fixed.
            // smoothJoysticks(&shoulder_manual);
            shoulder = shoulder_control*(precision_mode ? 0.6 : 1);
            winch = elbow_control*(precision_mode ? 0.6 : 1);
            
            shoulder = debuzz(clamp(shoulder, -1.0, 1.0));
            winch = debuzz(clamp(winch, -1.0, 1.0));
        }
        
//TODO: Feed-Forward
//TODO: Position macros (DPAD on second controller)
//TODO: Precision mode
//TODO: Rope tension
//TODO: Locks
//============================= Hopper ===========================
        if (intake_toggle)
        {
            intake = intake_reverse ? -1 : 1;
        }
        else
        {
            intake = 0;
        }
        if (hopper_tilt)
        {
            if (current_color)
                hand = hand_red_position;
            else
                hand = hand_blue_position;
        }
        else
        {
            hand = hand_level_position+hand_manual_control;
        }
        
        hand = clamp(hand, 0.0, 1.0);
        
        //for finding servo values
        // if(pad1.singlePress(B)) hand_level_position += 0.1;
        // if(pad1.singlePress(X)) hand_level_position -= 0.1;
        // hand_print_position = hand_level_position;

//TODO: Auto-score
//TODO: Block count
        //TODO: Tilt
//============================ Slide ===========================
        slide = 0.5;
        if (slide_right)
            slide = +slide_speed;
        if (slide_left)
            slide = -slide_speed;
        
        slide = clamp(slide, 0.0, 1.0);
//============================ Hook ===========================
        if(hook_toggle)
        {
            hook_left = hook_locked_position;
            hook_right = hook_locked_position;
        }
        else
        {
            hook_left = hook_level_position;
            hook_right = hook_level_position;
        }
        hook_left = clamp(hook_left, 0.0, 1.0);
        hook_right = clamp(hook_right, 0.0, 1.0);
//============================ Updates ===========================
        
        pad1.updateButtons(gamepad1);
        pad2.updateButtons(gamepad2);
        updateRobot();
    }
}
