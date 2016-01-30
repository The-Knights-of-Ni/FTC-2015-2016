#include "drive.h"
#include "arm.h"
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
#define shoulder_manual pad2stick1
#define elbow_manual pad2stick2
#define arm_stick ((v2f){pad2stick1.y, -pad2stick2.y})
#define arm_manual_toggle pad2.toggle(Y)
#define arm_score_mode_button pad2.press(DPAD_UP)
#define arm_intake_mode_button pad2.press(DPAD_DOWN)
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
                             "left_drive  = hardwareMap.dcMotor.get(\"leftd\");\n"
                             "right_drive = hardwareMap.dcMotor.get(\"rightd\");\n"
                             "shoulder    = hardwareMap.dcMotor.get(\"shoulder\");\n"
                             "winch       = hardwareMap.dcMotor.get(\"winch\");\n"
                             "intake      = hardwareMap.dcMotor.get(\"intake\");\n"
                             "right_drive.setDirection(DcMotor.Direction.REVERSE);\n"
                             "left_drive.setDirection(DcMotor.Direction.REVERSE);\n"
                             "shoulder.setDirection(DcMotor.Direction.REVERSE);\n"
                             "shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);\n"
                             "waitOneFullHardwareCycle();\n"
                             "intake.setDirection(DcMotor.Direction.REVERSE);\n"
                             "shoulder.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);\n"
                             "winch.setMode(DcMotorController.RunMode.RESET_ENCODERS);\n"
                             "waitOneFullHardwareCycle();\n"
                             "winch.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);\n"
                             "waitOneFullHardwareCycle();\n"
                             "//shoulder.setDirection(DcMotor.Direction.REVERSE);\n"
                             "//elbow.setDirection(DcMotor.Direction.REVERSE);\n"
                             "\n"
                             "hand = hardwareMap.servo.get(\"hand\");\n"
                             "slide = hardwareMap.servo.get(\"slide\");\n"
                             "hook_left = hardwareMap.servo.get(\"hook_left\");\n"
                             "hook_right = hardwareMap.servo.get(\"hook_right\");\n"
                             "hook_left.setDirection(Servo.Direction.REVERSE);");
    
    jni_misc_string = (
        "public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.\n"
        "{\n"
        "    ByteBuffer stick = ByteBuffer.allocate(45);\n"
        "    stick.put(joystick);\n"
        "    return stick.getInt(40);//Offset value\n"
        "}\n");
    
    ptime = jniDoubleIn("return time;");
    pright_drive_encoder = jniIntIn("return right_drive.getCurrentPosition();");
    pleft_drive_encoder = jniIntIn("return left_drive.getCurrentPosition();");
    pwinch_encoder = jniIntIn("return winch.getCurrentPosition();");
    pshoulder_encoder = jniIntIn("return shoulder.getCurrentPosition();");
    int * current_color = jniIntIn("return (FtcRobotControllerActivity.red ? 1 : 0);");
    
    jniOut("left_drive.setPower(", pleft_drive, ");;");
    jniOut("right_drive.setPower(", pright_drive, ");;");
    jniOut("winch.setPower(", pwinch, ");;");
    jniOut("shoulder.setPower(", pshoulder, ");;");
    jniOut("intake.setPower(", pintake, ");;");
    
    jniOut("hand.setPosition(", phand,");");
    jniOut("slide.setPosition(", pslide,");");
    jniOut("hook_left.setPosition(", phook_left,");");
    jniOut("hook_right.setPosition(", phook_right,");");

    float * pshoulder_print_theta;
    #define shoulder_print_theta (*pshoulder_print_theta)
    jniOut("telemetry.addData(\"shoulder theta\", ", pshoulder_print_theta,");");
    
    float * pforearm_print_theta;
    #define forearm_print_theta (*pforearm_print_theta)
    jniOut("telemetry.addData(\"forearm theta\", ", pforearm_print_theta,");");
    
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
    
    waitForStart();
    
    float dt;
    float old_time = time;
    
    interruptable for ever
    {
        dt = time - old_time;
        old_time = time;
        
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
        deadZone(drive_stick);
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
        //TODO: correctly convert to angle
        elbow_potentiometer_angle = lerp(
            (360-((180.0f-potentiometer_range*0.5f+potentiometer_range*(elbow_potentiometer/(1023.0f)))+12.0f))*pi/180.0f,
            elbow_potentiometer_angle,
            exp(-500.0*dt));
        
        float new_shoulder_theta = shoulder_encoder/shoulder_gear_ratio/encoderticks_per_radian+pi*150/180.0;
        float new_inside_elbow_theta = elbow_potentiometer_angle;
        float new_winch_theta = winch_encoder/winch_gear_ratio/encoderticks_per_radian;
        shoulder_omega = lerp((new_shoulder_theta-shoulder_theta)/dt, shoulder_omega, 0.1);
        winch_omega = lerp((new_winch_theta-winch_theta)/dt, winch_omega, 0.1);
        float inside_elbow_omega = (new_inside_elbow_theta-inside_elbow_theta)/dt;
        
        shoulder_print_theta = target_inside_elbow_theta;
        forearm_print_theta = new_inside_elbow_theta;
        
        shoulder_theta = new_shoulder_theta;
        inside_elbow_theta = new_inside_elbow_theta;
        winch_theta = new_winch_theta;
        
        if(arm_manual_toggle) //IK
        {
            v2f target_arm_velocity = arm_stick;
            if(arm_score_mode_button)
            {
                score_mode = true;
                //target_shoulder_theta = 0.75;
                target_inside_elbow_theta = pi*2/5;
                
                shoulder = -clamp(2-0.6*(inside_elbow_theta-target_inside_elbow_theta), 0.0, 1.0);
                float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
                target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
                
                winch = shoulder*winch_gear_ratio/shoulder_gear_ratio;
            }
            else if(arm_intake_mode_button)
            {
                score_mode = false;
                //target_shoulder_theta = 2.75;
                target_inside_elbow_theta = 4.0;
                
                shoulder = clamp(2-0.6*(target_inside_elbow_theta-inside_elbow_theta), 0.0, 1.0);
                target_shoulder_theta = shoulder_theta;
                
                winch = shoulder*winch_gear_ratio/shoulder_gear_ratio;
            }
            else
            {
                if(score_mode)
                {
                    armAtVelocity(shoulder, winch,
                                  target_arm_theta, target_inside_elbow_theta, target_arm_velocity,
                                  shoulder_theta, inside_elbow_theta,
                                  shoulder_omega, dt);
                    
                    target_shoulder_theta = shoulder_theta;
                }
                else
                {
                    armJointsAtVelocity(shoulder, winch,
                                        target_shoulder_theta, target_inside_elbow_theta, target_arm_velocity,
                                        shoulder_theta, inside_elbow_theta,
                                        shoulder_omega, dt);
                    
                    float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
                    target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
                }
            }
            
            if(score_mode)
            {
                armToAngle(shoulder, winch,
                           target_arm_theta, target_inside_elbow_theta,
                           shoulder_theta, inside_elbow_theta,
                           score_mode, dt);
            }
            else
            {
                armJointsToAngle(shoulder, winch,
                                 target_shoulder_theta, target_inside_elbow_theta,
                                 shoulder_theta, inside_elbow_theta,
                                 score_mode, dt);
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
            
            deadZone(elbow_manual);
            deadZone(shoulder_manual);
            // smoothJoysticks(&elbow_manual);//This needs to be fixed.
            // smoothJoysticks(&shoulder_manual);
            shoulder = shoulder_manual.y*(precision_mode ? 0.6 : 1);
            winch = -elbow_manual.y*(precision_mode ? 0.6 : 1);
            
            shoulder = clamp(shoulder, -1.0, 1.0);
            winch = clamp(winch, -1.0, 1.0);
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
        pad1.updateButtons(gamepad1.buttons);
        pad2.updateButtons(gamepad2.buttons);
        updateRobot();
    }
}
