#include "white_rabbit.h"

#include "jni_functions.h"

#ifndef GENERATE
#undef jniMain
#define jniMain Java_com_qualcomm_ftcrobotcontroller_opmodes_Mk4Teleop_main //TODO: generate this
#endif

//TODO: Get RED/BLUE Status

float hook_level_position = 0.0f;
float hook_locked_position = 1.0f;
//KEYBINDS

//Drive
#define drive_stick ((v2f){pad1stick1.x, -pad1stick1.y})
#define drive_toggle pad1.toggle(LEFT_STICK_BUTTON)

//Hopper
#define hand_open_toggle pad2.singlePress(RIGHT_TRIGGER)
#define wrist_tilt_toggle pad2.singlePress(RIGHT_BUMPER)
bool8 wrist_tilt = 0;
float wrist_manual_control = 0;
#define wrist_manual_increase pad2.press(X)
#define wrist_manual_decrease pad2.press(B)

//Intake
#define intake_toggle pad1.toggle(LEFT_BUMPER)
#define intake_reverse pad1.press(RIGHT_BUMPER)

#define intake_out_toggle false //pad1.singlePress(A)
#define intake_tilt_manual (+pad1stick2.y)

//Arm
// #define shoulder_manual pad2stick1
// #define elbow_manual pad2stick2
//                       [0] shoulder, [1] winch/elbow
#define arm_stick ((v2f){pad2stick1.y, -pad2stick2.y})
#define arm_manual_toggle pad2.toggle(Y)
#define arm_score_mode_button pad2.singlePress(LEFT_TRIGGER)
#define arm_intake_mode_button pad2.singlePress(LEFT_BUMPER)
#define shoulder_precision_mode (!pad2.press(LEFT_STICK_BUTTON))
#define winch_precision_mode (false)//(pad2.press(RIGHT_STICK_BUTTON))

#define arm_slow_factor 0.4

//Hook
#define hook_toggle pad1.toggle(B)
#define score_hook_toggle pad1.toggle(RIGHT_TRIGGER)
    
//Climber Release    
#define climber_release_toggle pad1.toggle(A)

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
                            "Servo wrist;\n"
                            "Servo hook_left;\n"
                            "Servo hook_right;\n"
                            "Servo intake_tilt;\n"
                            "Servo score_hook;\n"
                            "/* End Motor Definitions */");
    
    jni_run_opmode_string = ("dim = hardwareMap.deviceInterfaceModule.get(\"dim\");\n"
                             "left_drive  = hardwareMap.dcMotor.get(\"leftd\");\n"
                             "right_drive = hardwareMap.dcMotor.get(\"rightd\");\n"
                             "shoulder    = hardwareMap.dcMotor.get(\"shoulder\");\n"
                             "winch       = hardwareMap.dcMotor.get(\"winch\");\n"
                             "intake      = hardwareMap.dcMotor.get(\"intake\");\n"
                             "//right_drive.setDirection(DcMotor.Direction.REVERSE);\n"
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
                             "shoulder.setDirection(DcMotor.Direction.REVERSE);\n"
                             "//winch.setDirection(DcMotor.Direction.REVERSE);\n"
                             "\n"
                             "hand = hardwareMap.servo.get(\"hand\");\n"
                             "hand.setDirection(Servo.Direction.REVERSE);\n"
                             "wrist = hardwareMap.servo.get(\"wrist\");\n"
                             "hook_left = hardwareMap.servo.get(\"hook_left\");\n"
                             "hook_right = hardwareMap.servo.get(\"hook_right\");\n"
                             "hook_left.setDirection(Servo.Direction.REVERSE);\n"
                             "intake_tilt = hardwareMap.servo.get(\"intake_tilt\");\n"
                             "//intake_tilt.setDirection(Servo.Direction.REVERSE);\n"
                             "score_hook = hardwareMap.servo.get(\"score_hook\");");
    
    jni_misc_string = (
        "public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.\n"
        "{\n"
        "    return ByteBuffer.wrap(joystick, 42, 4).getInt();\n"
        "}\n");
    
    ptime = jniDoubleIn("return time;");
    pright_drive_encoder = jniIntIn("return right_drive.getCurrentPosition();");
    pleft_drive_encoder = jniIntIn("return left_drive.getCurrentPosition();");
    pwinch_encoder = jniIntIn("return winch.getCurrentPosition();");
    pshoulder_encoder = jniIntIn("return shoulder.getCurrentPosition();");
    pelbow_potentiometer = jniIntIn("return dim.getAnalogInputValue(elbow_potentiometer_port);");
    pshoulder_potentiometer = jniIntIn("return dim.getAnalogInputValue(shoulder_potentiometer_port);");
    pcurrent_color = jniIntIn("return (FtcRobotControllerActivity.red ? 1 : 0);");
    
    pslider0 = jniIntIn("return FtcRobotControllerActivity.slider_0;");
    pslider1 = jniIntIn("return FtcRobotControllerActivity.slider_1;");
    pslider2 = jniIntIn("return FtcRobotControllerActivity.slider_2;");
    pslider3 = jniIntIn("return FtcRobotControllerActivity.slider_3;");
    
    pdim_digital_pins = jniIntIn("return dim.getDigitalInputStateByte();");
    
    jniOut("left_drive.setPower(", pleft_drive, ");");
    jniOut("right_drive.setPower(", pright_drive, ");");
    jniOut("winch.setPower(", pwinch, ");");
    jniOut("shoulder.setPower(", pshoulder, ");");
    jniOut("intake.setPower(", pintake, ");");
    
    jniOut("hand.setPosition(", phand,");");
    jniOut("wrist.setPosition(", pwrist,");");
    jniOut("hook_left.setPosition(", phook_left,");");
    jniOut("hook_right.setPosition(", phook_right,");");
    jniOut("intake_tilt.setPosition(", pintake_tilt,");");
    jniOut("score_hook.setPosition(", pscore_hook,");");
    
    //TODO: telemetry queue
    float * pshoulder_print_theta;
    #define shoulder_print_theta (*pshoulder_print_theta)
    jniOut("telemetry.addData(\"shoulder theta\", ", pshoulder_print_theta,");");
    
    float * pshoulder_compensation_print = 0;
    #define shoulder_compensation_print (*pshoulder_compensation_print)
    jniOut("telemetry.addData(\"shoulder_compensation\", ", pshoulder_compensation_print,");");
    
    float * pleft_drive_compensation_print = 0;
    #define left_drive_compensation_print (*pleft_drive_compensation_print)
    jniOut("telemetry.addData(\"left_drive_compensation\", ", pleft_drive_compensation_print,");");
    
    float * pright_drive_compensation_print = 0;
    #define right_drive_compensation_print (*pright_drive_compensation_print)
    jniOut("telemetry.addData(\"right_drive_compensation\", ", pright_drive_compensation_print,");");
    
    float * pleft_drive_print_theta = 0;
    #define left_drive_print_theta (*pleft_drive_print_theta)
    jniOut("telemetry.addData(\"left_drive_theta\", ", pleft_drive_print_theta,");");
    
    float * pright_drive_print_theta = 0;
    #define right_drive_print_theta (*pright_drive_print_theta)
    jniOut("telemetry.addData(\"right_drive_theta\", ", pright_drive_print_theta,");");

    int * pleft_drive_print_active = 0;
    #define left_drive_print_active (*pleft_drive_print_active)
    jniOut("telemetry.addData(\"left_drive_active\", ", pleft_drive_print_active,");");
    
    int * pshoulder_active_print = 0;
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
    
    float * parm_stage_print;
    #define arm_stage_print (*parm_stage_print)
    jniOut("telemetry.addData(\"arm stage\", ", parm_stage_print,");");
    
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
    
    wrist_tilt = 0;
    wrist_manual_control = 0;
    
     //TODO: figure out a better way to have things reset to their initial values
    arm_stage = 0;
    
    hand_time = 1000000;
    intake_time = 1000000;
    
    score_mode = true;
    
    zeroDriveSensors();
    
    waitForStart();
    
    robotStateIn();
    updateDriveSensors();
    left_drive_hold_theta = left_drive_theta;
    right_drive_hold_theta = right_drive_theta;
    
    // updateArmSensors();
    // float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
    //                                   -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
    // target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
    // target_shoulder_theta = shoulder_theta;
    // target_inside_elbow_theta = inside_elbow_theta;
    
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
        updateDriveSensors();
        
        auto drive_control = smoothJoysticks(drive_stick, 0, 0.2, 0.8, 1.0);
        
        if(!drive_toggle)
        {
            left_drive = -drive_control.y + drive_control.x;
            right_drive = -drive_control.y - drive_control.x;
        }
        else
        {
            left_drive = drive_control.y + drive_control.x;
            right_drive = drive_control.y - drive_control.x;
        }
        
        // if(!drive_control.dead)
        // {
        //     left_drive_active = 2;
        //     right_drive_active = 2;
        // }
        
        // armJointStabalizationFunction(&left_drive,
        //                               left_drive_theta, left_drive_omega,
        //                               &left_drive_active, &left_drive_compensation,
        //                               past_left_drive_thetas, &n_valid_left_drive_angles, left_drive_speed_threshold,
        //                               drive_kp, 0, drive_ki, drive_kslow,
        //                               &left_drive_hold_theta,
        //                               false);
        
        // armJointStabalizationFunction(&right_drive,
        //                               right_drive_theta, right_drive_omega,
        //                               &right_drive_active, &right_drive_compensation,
        //                               past_right_drive_thetas, &n_valid_right_drive_angles, right_drive_speed_threshold,
        //                               drive_kp, 0, drive_ki, drive_kslow,
        //                               &right_drive_hold_theta,
        //                               false);
        
        left_drive_compensation = clamp(left_drive_compensation, -1.0, 1.0);
        right_drive_compensation = clamp(right_drive_compensation, -1.0, 1.0);
        
        left_drive_compensation_print = left_drive_compensation;
        right_drive_compensation_print = right_drive_compensation;
        left_drive_print_theta = left_drive_theta;
        right_drive_print_theta = right_drive_theta;
        left_drive_print_active = left_drive_active;
        
        left_drive = clamp(left_drive, -1.0, 1.0);
        right_drive = clamp(right_drive, -1.0, 1.0);
        
//============================== Arm =============================
        updateArmSensors();
        
        shoulder_compensation_print = shoulder_compensation;
        shoulder_print_theta = shoulder_theta;
        forearm_print_theta = inside_elbow_theta;
        shoulder_active_print = shoulder_active;
        arm_stage_print = arm_stage;
        
        if(arm_manual_toggle) //IK
        {
            v2f target_arm_velocity = arm_stick;
            
            if(arm_score_mode_button)
            {
                score_mode = true;
                if(arm_stage != arm_idle) //cancel motion
                {
                    float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
                    target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
                    
                    arm_stage = arm_idle;
                }
                else arm_stage = arm_retracting;
            }
            else if(arm_intake_mode_button)
            {
                score_mode = false;
                if(arm_stage != arm_idle) //cancel motion
                {
                    target_shoulder_theta = shoulder_theta;
                    
                    arm_stage = arm_idle;
                }
                else arm_stage = arm_pre_preparing;
            }
            
            if(arm_stage != arm_idle)
            {
                armSwitchModes();
            }
            
            if(arm_stage == arm_idle) //since arm_stage can change in armSwitchModes()
            {
                arm_switching = 0;
                
                target_arm_velocity.x = filterArmJoystick(target_arm_velocity.x);
                target_arm_velocity.y = filterArmJoystick(target_arm_velocity.y);
                if(shoulder_precision_mode) target_arm_velocity.x *= arm_slow_factor;
                if(winch_precision_mode) target_arm_velocity.y *= arm_slow_factor;
                
                if(score_mode)
                {
                    armAtVelocity(target_arm_velocity);
                    
                    //set the unused target angle so it won't move when the mode is switched
                    target_shoulder_theta = shoulder_theta;
                    
                    armToPolarTarget();
                }
                else
                {
                    armJointsAtVelocity(target_arm_velocity);
                    
                    //set the unused target angle so it won't move when the mode is switched
                    float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
                    target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
                    
                    armToJointTarget();
                }
            }
            
            //clamp the integral factors to stop integral build up
            shoulder_compensation = clamp(shoulder_compensation, -1.0, 1.0);
            winch_compensation = clamp(winch_compensation, -1.0, 1.0);
            
            shoulder = clamp(shoulder, -1.0, 1.0);
            winch = clamp(winch, -1.0, 1.0);
            if(winch > 0.0 && !tension_switch && inside_elbow_theta > pi) winch = 0.0;
        }
        else //Manual
        {
            //set targets to current position so it will remember the current value when arm control is enabled
            float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                              -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
            target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
            target_shoulder_theta = shoulder_theta;
            target_inside_elbow_theta = inside_elbow_theta;
            
            float shoulder_control = filterArmJoystick(arm_stick[0]);
            float winch_control = filterArmJoystick(arm_stick[1]);
            
            if(shoulder_precision_mode) shoulder_control *= arm_slow_factor;
            if(winch_precision_mode) winch_control *= arm_slow_factor;
            
            shoulder = shoulder_control;
            winch = winch_control;
            
            //filter low power to remove annoying buzzing noise and ensure the motor powers remain in bounds
            shoulder = debuzz(clamp(shoulder, -1.0, 1.0));
            winch = debuzz(clamp(winch, -1.0, 1.0));
        }
        
//TODO: Feed-Forward
//TODO: Precision mode
//TODO: Rope tension
//TODO: Locks
        
//====================== Intake & Hopper =======================
        if (intake_toggle)
        {
            intake = intake_reverse ? -1 : 1;
        }
        else
        {
            intake = 0;
        }
        
        if(armOnIntakeSide())
        {
            setIntakeOut();
        }
        else if(intake_out_toggle)
        {
            intake_out = !intake_out;
            intake_time = 0;
        }
        doIntake();
        intake_tilt += intake_tilt_manual/2.0;
        
        if(wrist_tilt_toggle) wrist_tilt = !wrist_tilt;
        
        if(shoulder_theta > 1.3 && inside_elbow_theta > pi)
        {
            wrist_tilt = false;
        }
        
        if(wrist_tilt)
        {
            if(current_color) wrist = wrist_red_position;
            else              wrist = wrist_blue_position;
        }
        else
        {
            wrist = wrist_level_position;
            wrist_manual_control = 0;
        }
        wrist_manual_control += 0.5*((int) wrist_manual_increase - (int)wrist_manual_decrease)*dt;
        wrist += wrist_manual_control;
        
        if(hand_open_toggle)
        {
            hand_open = !hand_open;
            hand_time = 0;
        }
        doHand();
        
        intake_tilt = clamp(intake_tilt, 0.0, 1.0);
        wrist = clamp(wrist, 0.0, 1.0);
        hand = clamp(hand, 0.0, 1.0);
        
        //for finding servo values
        // if(pad1.singlePress(B)) hand_level_position += 0.1;
        // if(pad1.singlePress(X)) hand_level_position -= 0.1;
        // hand_print_position = hand_level_position;
        
        //TODO: Auto-score
        //TODO: Block count
        //TODO: Tilt
        
//============================= Hook =============================
        if(hook_toggle)
        {
            hook_left = hook_locked_position;
        }
        else
        {
            hook_left = hook_level_position;
        }
        hook_left = clamp(hook_left, 0.0, 1.0);
        
//=========================Score Hook ============================
        if(score_hook_toggle)
        {
            score_hook = 1.0;
        }
        else
        {
            score_hook = 0.0;
        }
        score_hook = clamp(score_hook, 0.0, 1.0);
        
//========================= Climber Release ======================
        if(climber_release_toggle)
        {
            //TODO: rename to climber release
            hook_right = 1.0;
        }
        else
        {
            hook_right = 0.0;
        }
        hook_right = clamp(hook_right, 0.0, 1.0);
        
//============================ Updates ===========================
        
        pad1.updateButtons(gamepad1);
        pad2.updateButtons(gamepad2);
        updateRobot();
    }
}
