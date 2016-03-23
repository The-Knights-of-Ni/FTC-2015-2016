#include "white_rabbit.h"

#include "jni_functions.h"

#ifndef GENERATE
#undef jniMain
#define jniMain Java_com_qualcomm_ftcrobotcontroller_opmodes_Mk4Teleop_main //TODO: generate this
#endif

//TODO: Get RED/BLUE Status

float hook_right_level_position = 70.0f/255.0f;
float hook_right_locked_position = 290.0f/255.0f;
float hook_left_level_position = 60.0f/255.0f;
float hook_left_locked_position = 180.0f/255.0f;
//KEYBINDS

//Drive
#define drive_stick ((v2f){pad1stick1.x, -pad1stick1.y})
#define drive_toggle pad1.toggle(LEFT_STICK_BUTTON)

//Hopper
#define hand_open_toggle pad2.singlePress(RIGHT_TRIGGER)
#define wrist_tilt_toggle pad2.singlePress(RIGHT_BUMPER)
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

#define pullup_button pad2.singlePress(A)

#define arm_slow_factor 0.4

//Hook
#define hook_toggle pad1.singlePress(B)
#define score_hook_toggle pad1.toggle(RIGHT_TRIGGER)
    
//Climber Release    
#define climber_release_toggle pad1.toggle(A)

float drive_straight_component = 0;

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
                         "import com.qualcomm.robotcore.hardware.I2cDevice;\n"
                         "import com.qualcomm.robotcore.hardware.Servo;\n"
                         "import com.qualcomm.robotcore.hardware.VoltageSensor;\n"
                         logging_jni_import_string);
    
    //TODO: shortcut for defining and declaring motors, servos, etc.
    jni_variables_string = ("/* Start Motor Definitions */\n"
                            "VoltageSensor left_drive_voltage;\n"
                            "VoltageSensor right_drive_voltage;\n"
                            "DeviceInterfaceModule dim;\n"
                            "IMU imu;"
                            "int elbow_potentiometer_port = 7;\n"
                            "int shoulder_potentiometer_port = 1;\n"
                            "int intake_potentiometer_port = 5;\n"
                            "int wrist_potentiometer_port = 3;\n"
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
                            "/* End Motor Definitions */"
                            logging_jni_misc_string);
    
    jni_run_opmode_string = ("left_drive_voltage = hardwareMap.voltageSensor.get(\"Left Drive + Shoulder\");\n"
                             "right_drive_voltage = hardwareMap.voltageSensor.get(\"Intake + Right Drive\");\n"
                             "dim = hardwareMap.deviceInterfaceModule.get(\"dim\");\n"
                             "I2cDevice imu_i2c_device = hardwareMap.i2cDevice.get(\"imu\");\n"
                             "imu = new IMU(imu_i2c_device, this);\n"
                             "int error = imu.init(IMU.mode_ndof,\n"
                             "        (byte) (IMU.units_acc_m_per_s2 |\n"
                             "                IMU.units_angle_deg |\n"
                             "                IMU.units_angular_vel_deg_per_s |\n"
                             "                IMU.units_temp_C |\n"
                             "                IMU.units_pitch_convention_android));\n"
                             "if (error != 0) {\n"
                             "    for (; ; ) {\n"
                             "        telemetry.addData(\"error initializing imu\", 0);\n"
                             "        waitOneFullHardwareCycle();\n"
                             "    }\n"
                             "}\n"
                             "imu.vel_x = 0.0f;\n"
                             "imu.vel_y = 0.0f;\n"
                             "imu.vel_z = 0.0f; \n"
                             "\n"
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
                             "wrist.setDirection(Servo.Direction.REVERSE);\n"
                             "hook_left = hardwareMap.servo.get(\"hook_left\");\n"
                             "hook_right = hardwareMap.servo.get(\"hook_right\");\n"
                             "hook_left.setDirection(Servo.Direction.REVERSE);\n"
                             "intake_tilt = hardwareMap.servo.get(\"intake_tilt\");\n"
                             "intake_tilt.setDirection(Servo.Direction.REVERSE);\n"
                             "score_hook = hardwareMap.servo.get(\"score_hook\");\n"
                             "telemetry.addData(\"ready\", \"\");\n");
    
    jni_misc_string = (
        "public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.\n"
        "{\n"
        "    return ByteBuffer.wrap(joystick, 42, 4).getInt();\n"
        "}\n"
        "\n"
        "public void zeroIMU()\n"
        "{\n"
        "    imu.rezero();\n"
        "}\n");
    
    ptime = jniDoubleIn("return time;");
    pright_drive_encoder = jniIntIn("return right_drive.getCurrentPosition();");
    pleft_drive_encoder = jniIntIn("return left_drive.getCurrentPosition();");
    pwinch_encoder = jniIntIn("return winch.getCurrentPosition();");
    pshoulder_encoder = jniIntIn("return shoulder.getCurrentPosition();");
    pelbow_potentiometer = jniIntIn("return dim.getAnalogInputValue(elbow_potentiometer_port);");
    pshoulder_potentiometer = jniIntIn("return dim.getAnalogInputValue(shoulder_potentiometer_port);");
    pintake_potentiometer = jniIntIn("return dim.getAnalogInputValue(intake_potentiometer_port);");
    pwrist_potentiometer = jniIntIn("return dim.getAnalogInputValue(wrist_potentiometer_port);");
    pleft_drive_voltage = jniFloatIn("return (float)left_drive_voltage.getVoltage();");
    pright_drive_voltage = jniFloatIn("return (float)right_drive_voltage.getVoltage();");
    
    pcurrent_color = jniIntIn("return (FtcRobotControllerActivity.red ? 1 : 0);");
    
    pslider0 = jniIntIn("return FtcRobotControllerActivity.slider_0;");
    pslider1 = jniIntIn("return FtcRobotControllerActivity.slider_1;");
    pslider2 = jniIntIn("return FtcRobotControllerActivity.slider_2;");
    pslider3 = jniIntIn("return FtcRobotControllerActivity.slider_3;");
    
    pdim_digital_pins = jniIntIn("return dim.getDigitalInputStateByte();");
    
    pimu_values = jniStructIn(
        imu_state,
        "if(imu.checkForUpdate()) {\n"
        "    return {imu.eul_x, imu.eul_y, imu.eul_z, imu.gyr_x, imu.gyr_y, imu.gyr_z, imu.vel_x, imu.vel_y, imu.vel_z};\n"
        "}\n");
    short * pimu_heading = &(pimu_values->orientation.x);
    jniOut("telemetry.addData(\"imu heading\", ", pimu_heading, "/16.0);");
    short * pimu_tilt = &(pimu_values->orientation.y);
    jniOut("telemetry.addData(\"imu tilt\", ", pimu_tilt, "/16.0);");
    short * pimu_roll = &(pimu_values->orientation.z);
    jniOut("telemetry.addData(\"imu roll\", ", pimu_roll, "/16.0);");
    
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

    float * pdrive_direction_print;
    #define drive_direction_print (*pdrive_direction_print)
    jniOut("telemetry.addData(\"drive direction\", ", pdrive_direction_print,");");

    float * pdrive_theta_print;
    #define drive_theta_print (*pdrive_theta_print)
    jniOut("telemetry.addData(\"drive theta\", ", pdrive_theta_print,");");
    
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
    
    initLogfile();
    
    Button pad1 = {};
    Button pad2 = {};
    v2f pad1stick1;
    v2f pad1stick2;
    v2f pad2stick1;
    v2f pad2stick2;
    
    target_arm_theta = pi*7/12;
    target_shoulder_theta = pi*150/180;
    target_inside_elbow_theta = pi/6;
    
    armFunction = armUserControl;
    
    shoulder_omega = 0;
    winch_omega = 0;
    inside_elbow_omega = 0;
    
    shoulder_compensation = 0;
    
    arm_tilt_adjustment = 0;
    arm_tilt_omega_adjustment = 0;
    
    wrist_tilt = 0;
    wrist_manual_control = 0;
    wrist_time = 0;
    
    //TODO: figure out a better way to have things reset to their initial values
    
    hand_time = 1000000;
    
    score_mode = true;
    
    setIntakeOut();
    
    #ifndef GENERATE
    jmethodID imu_rezero_id;
    jobject imu_object;
    {//get imu.rezero() method id
        jclass imu_class = env->FindClass("com/qualcomm/ftcrobotcontroller/opmodes/IMU");
        imu_rezero_id = env->GetMethodID(imu_class, "rezero", "()V");
        imu_object = env->GetObjectField(self, env->GetFieldID(cls, "imu", "Lcom/qualcomm/ftcrobotcontroller/opmodes/IMU;"));
    }
    #endif
    
    drive_straight_component = 0;
    
    waitForStart();
    
    zeroDriveSensors();
    
    robotStateIn();
    
    imu_orientation_offsets = (v3f){pimu_values->orientation.x, pimu_values->orientation.y, pimu_values->orientation.z};;
    
    #ifndef GENERATE
    env->CallVoidMethod(imu_object, imu_rezero_id); //rezero imu
    #endif
    
    robotStateIn();
    updateDriveSensors();
    left_drive_hold_theta = left_drive_theta;
    right_drive_hold_theta = right_drive_theta;
    
    // updateArmSensors();
    // float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
    //                                   -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
    // target_arm_theta = shoulder_theta-asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta));
    // target_arm_y = shoulder_axis_to_end*cos(target_arm_theta-vertical_arm_theta);
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

//============================== IMU =============================        
        updateIMU();
        
//============================= Drive ============================
        updateDriveSensors();
        
        v2f drive_control = smoothJoysticks254Style(drive_stick);
        
        #if 0
        #if 0
        if(fabs(drive_control.x) > deadzone_radius)
        {
            float max_turn_power = 1.0;//-fabs(drive_control.y);
            float target_turn_power = max_turn_power*drive_control.x;
            float max_turn_omega = pi;//sprocket_pitch_radius*neverest_max_omega/drive_gear_ratio/drive_radius;
            float target_heading_omega = target_turn_power*max_turn_omega;
            float heading_omega_error = target_heading_omega-imu_heading_omega*pi/180.0;
            heading_omega_ifactor += 10.0*heading_omega_error*dt;
            left_drive  = +(target_turn_power + heading_omega_ifactor);
            right_drive = -(target_turn_power + heading_omega_ifactor);
        }
        else
        {
            left_drive = 0;
            right_drive = 0;
            heading_omega_ifactor = 0;
        }
        #else
        left_drive  = +drive_control.x;
        right_drive = -drive_control.x;
        #endif
        
        if(!drive_toggle)
        {
            left_drive -= drive_control.y;
            right_drive -= drive_control.y;
        }
        else
        {
            left_drive += drive_control.y;
            right_drive += drive_control.y;
        }

        #else //discretized drive control
        
        float drive_control_norm = norm(drive_control);
        float drive_control_theta = atan2(drive_control.y, drive_control.x);
        if(drive_control_theta < 0) drive_control_theta += 2*pi;
        int octant =
            +(drive_control_theta > pi* 2/12)
            +(drive_control_theta > pi* 5/12)
            +(drive_control_theta > pi* 7/12)
            +(drive_control_theta > pi*10/12)
            +(drive_control_theta > pi*14/12)
            +(drive_control_theta > pi*17/12)
            +(drive_control_theta > pi*19/12)
            +(drive_control_theta > pi*22/12);
        octant = octant % 8;
        
        float target_straight_component = (drive_toggle ? 1.0 : -1.0)*(((octant>>2)&1) ? -1.0 : 1.0);
        
        float turn_component = 0.0;
        
        if(octant&1)
        { //diagonal
            turn_component = (((((octant>>1)^(octant>>2)))&1) ? -0.75 : 0.75);
        }
        else
        { //straight
            if(((octant>>1)&1) == 0)
            {
                target_straight_component = 0;
                turn_component = (((octant>>2)&1) ? -1.0 : 1.0);
            }
        }
        
        target_straight_component *= drive_control_norm;
        turn_component            *= drive_control_norm;

        drive_straight_component = target_straight_component;
        // float max_accel = 10.0;
        // if(fabs(target_straight_component - drive_straight_component) < max_accel*dt)
        // {
        //     drive_straight_component = target_straight_component;
        // }
        // else
        // {
        //     drive_straight_component += sign(target_straight_component - drive_straight_component)*max_accel*dt;
        // }
        
        left_drive  = drive_straight_component + turn_component;
        right_drive = drive_straight_component - turn_component;
        
        drive_direction_print = octant;
        drive_theta_print = drive_control_theta;
        
        #endif
        
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
        
        if(arm_manual_toggle) //IK
        {
            target_arm_velocity = arm_stick;
            
            if(arm_score_mode_button)
            {
                score_mode = true;
                if(armFunction != armUserControl) //cancel motion
                {
                    float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
                    target_arm_theta = shoulder_theta-asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta));
                    target_arm_y = shoulder_axis_to_end*cos(target_arm_theta-vertical_arm_theta);
                    
                    armFunction = armUserControl;
                }
                else
                {
                    arm_line = 0;
                    armFunction = armToScoreMode;
                }
            }
            else if(arm_intake_mode_button)
            {
                if(armFunction != armUserControl) //cancel motion
                {
                    target_shoulder_theta = shoulder_theta;
                    
                    armFunction = armUserControl;
                }
                else
                {
                    arm_line = 0;
                    armFunction = armToIntakeMode;
                }
            }
            else if(pullup_button)
            {
                if(armFunction != armUserControl) //cancel motion
                {
                    target_shoulder_theta = shoulder_theta;
                    float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
                    target_arm_theta = shoulder_theta-asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta));
                    target_arm_y = shoulder_axis_to_end*cos(target_arm_theta-vertical_arm_theta);
                    
                    armFunction = armUserControl;
                }
                else
                {
                    arm_line = 0;
                    armFunction = armPullup;
                }                
            }
            
            target_arm_velocity.x = filterArmJoystick(target_arm_velocity.x);
            target_arm_velocity.y = filterArmJoystick(target_arm_velocity.y);
            if(shoulder_precision_mode) target_arm_velocity.x *= arm_slow_factor;
            if(winch_precision_mode) target_arm_velocity.y *= arm_slow_factor;
            
            armFunction();
            
            //clamp the integral factors to stop integral build up
            shoulder_compensation = clamp(shoulder_compensation, -1.0, 1.0);
            winch_compensation = clamp(winch_compensation, -1.0, 1.0);
            
            shoulder = clamp(shoulder, -1.0, 1.0);
            winch = clamp(winch, -1.0, 1.0);
            
            shoulder *= 12.0/left_drive_voltage;
            winch *= 12.0/left_drive_voltage;
            
            shoulder = clamp(shoulder, -1.0, 1.0);
            winch = clamp(winch, -1.0, 1.0);
            
            log("shoulder: %f. winch %f, shoulder_theta: %f, inside_elbow_theta: %f, intake_theta: %f, arm_line: %d\n",
                shoulder,      winch,    shoulder_theta,     inside_elbow_theta,     intake_theta,     arm_line);
            if(winch > 0.0 && !tension_switch && inside_elbow_theta > pi) winch = 0.0;
        }
        else //Manual
        {
            //set targets to current position so it will remember the current value when arm control is enabled
            float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                              -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
            target_arm_theta = shoulder_theta-asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta));
            target_arm_y = shoulder_axis_to_end*cos(target_arm_theta-vertical_arm_theta);
            target_shoulder_theta = shoulder_theta;
            target_inside_elbow_theta = inside_elbow_theta;
            
            shoulder_compensation = 0;
            arm_tilt_adjustment = 0;
            arm_tilt_omega_adjustment = 0;
            
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
        
        if(hook_toggle)
        {
            if(intake_out)
            {
                setIntakeIn();
            }
            else
            {
                setIntakeOut();
            }
        }
        
        target_intake_theta = clamp(target_intake_theta, intake_out_theta, 2.0);

        if(fabs(intake_tilt_manual) > deadzone_radius)
        {
            intake_tilt = -intake_tilt_manual;
            target_intake_theta = intake_theta;
        }
        else
        {
            doIntake();
        }
        
        if(wrist_tilt_toggle) wrist_tilt = !wrist_tilt;
        
        if(shoulder_theta > 1.3 && inside_elbow_theta > pi)
        {
            wrist_tilt = false;
        }
        
        wrist_manual_control += 0.5*((int) wrist_manual_increase - (int)wrist_manual_decrease)*dt;
        doWrist();
        
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
        if(!intake_out)
        {
            hook_left = hook_left_locked_position;
            hook_right = hook_right_locked_position;

        }
        else
        {
            hook_left = hook_left_level_position;
            hook_right = hook_right_level_position;
        }
        hook_left = clamp(hook_left, 0.0, 1.0);
        hook_right = clamp(hook_right, 0.0, 1.0);
        
//======================== Pullup Hook ============================
        if(score_hook_toggle)
        {
            score_hook = 0.0;
        }
        else
        {
            score_hook = 1.0;
        }
        score_hook = clamp(score_hook, 0.0, 1.0);
        
// //========================= Climber Release ======================
//         if(climber_release_toggle)
//         {
//             //TODO: rename to climber release
//             hook_right = 1.0;
//         }
//         else
//         {
//             hook_right = 0.0;
//         }
//         hook_right = clamp(hook_right, 0.0, 1.0);
        
//============================ Updates ===========================
        
        pad1.updateButtons(gamepad1);
        pad2.updateButtons(gamepad2);
        updateRobot();
    }
}
