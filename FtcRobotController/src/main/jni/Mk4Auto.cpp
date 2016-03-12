#include "autonomous.h" //NOTE: needs customAutonomousUpdate must be declared before including this
#include "vision.h"

#include "jni_functions.h"

#include <stdio.h>
#include <unistd.h>
FILE * log_file;

static jmp_buf deploy_jump;

float start_time = 0;
//stuff that need to be constantly updated in the background but is not intensive enough to deserve a seperate thread
void Mk4AutonomousUpdate()
{
    //if(time-start_time > 20.0) longjmp(exit_jump, 1);
    
    //TODO: make this sensor filter stuff a function in arm.h    
    shoulder = 0;
    winch = 0;
    
    armFunction();
    if(armFunction == armUserControl)
    {
        armFunction = armAutonomousControl;
    }
    
    doIntake();
    doHand();
    doWrist();
    
    //clamp the integral factors to stop integral build up
    shoulder_compensation = clamp(shoulder_compensation, -1.0, 1.0);
    winch_compensation = clamp(winch_compensation, -1.0, 1.0);
    
    shoulder = clamp(shoulder, -1.0, 1.0);
    winch = clamp(winch, -1.0, 1.0);
    if(winch > 0.0 && !tension_switch) winch = 0.0;
    
    left_drive *= 14.0/left_drive_voltage;
    right_drive *= 14.0/right_drive_voltage;
    
    left_drive = clamp(left_drive, -1.0, 1.0);
    right_drive = clamp(right_drive, -1.0, 1.0);
    intake = clamp(intake, -1.0, 1.0);
    
    intake_tilt = clamp(intake_tilt, 0.0, 1.0);
    wrist = clamp(wrist, 0.0, 1.0);
    hand = clamp(hand, 0.0, 1.0);
    hook_left = clamp(hook_left, 0.0, 1.0);
    hook_right = clamp(hook_right, 0.0, 1.0);
    
    if(log_file) fprintf(log_file, "%f %f %f  %f %f %f  %d %d  %f %f",  imu_heading, imu_tilt, imu_roll, imu_vel.x, imu_vel.y, imu_vel.z, left_drive_encoder, right_drive_encoder, left_drive, right_drive);
    
    // intake = 0;
    if(suppress_arm)
    {
        shoulder = 0;
        winch = 0;
    }
    // //intake_tilt = continuous_servo_stop;
    // wrist = wrist_level_position;
    // hand = 0;
    // hook_left = 0;
    // hook_right = 0;
}

#ifndef GENERATE
#undef jniMain
#define jniMain Java_com_qualcomm_ftcrobotcontroller_opmodes_Mk4Auto_main
#endif

extern "C"
void jniMain(JNIEnv * _env, jobject _self)
{
    //NOTE: DON'T FORGET THESE //TODO: make it so you don't need these
    env = _env;
    self = _self;
    initJNI();
    
    // exiting = false;
    // {        
    //     jclass class_env = env->FindClass(env, "android/os/Environment");
    //     if(!class_env)
    //     {
    //         cleanupJNI();
    //         exiting = true;
    //         goto interuptable_goto;
    //     }
    //     jmethodID getExternalStorageDirectoryID = env->GetStaticMethod(env, class_env,
    //                                                                         "getExternalStorageDirectory",
    //                                                                         "()Ljava/io/File;");
    //     if(!getExternalStorageDirectoryID)
    //     {
    //         cleanupJNI();
    //         exiting = true;
    //         goto interuptable_goto;
    //     }
    //     jobject log_file_object = env->CallStaticObjectMethod(env, class_env, getExternalStorageDirectoryID);
    //     auto exception = env->ExceptionOccurred(env);
    //     if(exception)
    //     {
    //         env->ExceptionDescribe(env);
    //         env->ExceptionClear(env);
    //         cleanupJNI();
    //         exiting = true;
    //         goto interuptable_goto;
    //     }
    //     jclass log_file_class = env->GetObjectClass(env, log_file_object);
    //     if(!log_file_class)
    //     {
    //         cleanupJNI();
    //         exiting = true;
    //         goto interuptable_goto;
    //     }
    //     jmethodIDgetAbsolutePath = env->GetMethodId(env, log_file_object, "getAb");
    // }
    
    customAutonomousUpdate = Mk4AutonomousUpdate;
    
    jni_import_string = (
        "import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;\n"
        "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n"
        "import com.qualcomm.robotcore.exception.RobotCoreException;\n"
        "import com.qualcomm.robotcore.hardware.DcMotor;\n"
        "import com.qualcomm.robotcore.hardware.DcMotorController;\n"
        "import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;\n"
        "import com.qualcomm.robotcore.hardware.I2cDevice;\n"
        "import com.qualcomm.robotcore.hardware.Servo;\n"
        "import com.qualcomm.robotcore.hardware.VoltageSensor;\n"
        "import android.hardware.Camera;\n"
        "import android.graphics.ImageFormat;\n");
    
    //TODO: shortcut for defining and declaring motors, servos, etc.
    jni_variables_string = (
        "/* Start Motor Definitions */\n"
        "VoltageSensor left_drive_voltage;\n"
        "VoltageSensor right_drive_voltage;\n"
        "DeviceInterfaceModule dim;\n"
        "IMU imu;"
        "int elbow_potentiometer_port = 7;\n"
        "int shoulder_potentiometer_port = 1;\n"
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
        "/* End Motor Definitions */");
    
    jni_run_opmode_string = (
        "left_drive_voltage = hardwareMap.voltageSensor.get(\"Left Drive + Shoulder\");\n"
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
        "hook_left = hardwareMap.servo.get(\"hook_left\");\n"
        "hook_right = hardwareMap.servo.get(\"hook_right\");\n"
        "hook_left.setDirection(Servo.Direction.REVERSE);\n"
        "intake_tilt = hardwareMap.servo.get(\"intake_tilt\");\n"
        "//intake_tilt.setDirection(Servo.Direction.REVERSE);"
        "\n"
        "dim.setLED(0, false);\n"
        "dim.setLED(1, false);\n"
        "while (!FtcRobotControllerActivity.aligned || (!FtcRobotControllerActivity.red && !FtcRobotControllerActivity.blue))\n"
        "{\n"
        "    telemetry.addData(\"unchecked boxes\", \"fix it\");\n"
        "    waitForNextHardwareCycle();\n"
        "}\n"
        "if(FtcRobotControllerActivity.red)\n"
        "{\n"
        "    dim.setLED(0, false);\n"
        "    dim.setLED(1, true);\n"
        "}\n"
        "else if(FtcRobotControllerActivity.blue)\n"
        "{\n"
        "    dim.setLED(0, true);\n"
        "    dim.setLED(1, false);\n"
        "}\n"
        "else\n"
        "{\n"
        "    dim.setLED(0, false);\n"
        "    dim.setLED(1, false);\n"
        "}"
        "telemetry.addData(\"ready\", \"\");\n"
        "waitForStart();\n"
        "imu.rezero();\n");
    
    jni_misc_string = (
        "Camera camera = null;\n"
        "int camera_w = 0;\n"
        "int camera_h = 0;\n"
        "CameraPreviewCallback camera_preview_callback;\n"
        "\n"
        "byte[] camera_buffer = null;\n"
        "\n"
        "class CameraPreviewCallback implements Camera.PreviewCallback\n"
        "{\n"
        "    \n"
        "    \n"
        "    CameraPreviewCallback(){}\n"
        "    public void onPreviewFrame(byte[] data, Camera camera)\n"
        "    {\n"
        "        camera.addCallbackBuffer(camera_buffer);\n"
        "    }\n"
        "}\n");
    
    jni_constructor_string = ("camera = FtcRobotControllerActivity.camera_preview.camera;\n"
                              "Camera.Parameters parameters = camera.getParameters();\n"
                              "Camera.Size camera_size = parameters.getPreviewSize();\n"
                              "camera_w = camera_size.width;\n"
                              "camera_h = camera_size.height;\n"
                              "\n"
                              "camera_buffer = new byte[camera_w*camera_h*4];\n"
                              "camera_preview_callback = new CameraPreviewCallback();\n"
                              "\n"
                              "camera.setPreviewCallbackWithBuffer(camera_preview_callback);\n"
                              "camera.addCallbackBuffer(camera_buffer);\n"
                                "parameters.setPreviewFormat(ImageFormat.NV21);\n"
                                "parameters.setExposureCompensation(0);\n"
                                "parameters.setWhiteBalance(Camera.Parameters.WHITE_BALANCE_INCANDESCENT);\n"
                               "parameters.set(\"iso\", \"ISO100\");\n"
                                  "parameters.set(\"max-exposure-time\", 2000000);\n"
                                   "parameters.set(\"min-exposure-time\", 2000000);\n"
                                   "        DbgLog.error(\"Camera parameters: \"+parameters.flatten());");

    ptime = jniDoubleIn("return time;");
    pright_drive_encoder = jniIntIn("return right_drive.getCurrentPosition();");
    pleft_drive_encoder = jniIntIn("return left_drive.getCurrentPosition();");
    pwinch_encoder = jniIntIn("return winch.getCurrentPosition();");
    pshoulder_encoder = jniIntIn("return shoulder.getCurrentPosition();");
    pelbow_potentiometer = jniIntIn("return dim.getAnalogInputValue(elbow_potentiometer_port);");
    pshoulder_potentiometer = jniIntIn("return dim.getAnalogInputValue(shoulder_potentiometer_port);");
    pleft_drive_voltage = jniFloatIn("return (float)left_drive_voltage.getVoltage();");
    pright_drive_voltage = jniFloatIn("return (float)right_drive_voltage.getVoltage();");
    
    pdim_digital_pins = jniIntIn("return dim.getDigitalInputStateByte();");
    
    pimu_values = jniStructIn(
        imu_state,
        "if(imu.checkForUpdate()) {\n"
        "    return {imu.eul_x, imu.eul_y, imu.eul_z, imu.vel_x, imu.vel_y, imu.vel_z};\n"
        "}\n");
    
    pcurrent_color = jniIntIn("return (FtcRobotControllerActivity.red ? 1 : 0);");
    
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
    
    jniOut("telemetry.addData(\"Indicator:\", ", pindicator, ");");
    jniOut("telemetry.addData(\"left_drive_encoder:\", ", pleft_drive_encoder, ");");
    jniOut("telemetry.addData(\"right_drive_encoder:\", ", pright_drive_encoder, ");");
    jniOut("telemetry.addData(\"beacon right:\", (", pbeacon_right," == 1 ? \"red\" : \"blue\"));");
    short * pimu_heading = &(pimu_values->orientation.x);
    jniOut("telemetry.addData(\"heading:\", ", pimu_heading, ");");
    
    jniOut("telemetry.addData(\"target time:\", ", pdrive_time, ");");
    jniOut("telemetry.addData(\"acceleration time:\", ", pacceleration_time, ");");
    
    pslider0 = jniIntIn("return FtcRobotControllerActivity.slider_0;");
    pslider1 = jniIntIn("return FtcRobotControllerActivity.slider_1;");
    pslider2 = jniIntIn("return FtcRobotControllerActivity.slider_2;");
    pslider3 = jniIntIn("return FtcRobotControllerActivity.slider_3;");
    
    jniOut("telemetry.addData(\"slider 0\", ", pslider0,");");
    jniOut("telemetry.addData(\"slider 1\", ", pslider1,");");
    jniOut("telemetry.addData(\"slider 2\", ", pslider2,");");
    jniOut("telemetry.addData(\"slider 3\", ", pslider3,");");
    
    jniGenerate();
    
    intake_out = true;
    intake_time = 1000;
    wrist = wrist_level_position;
    wrist_tilt = false;
    wrist_time = 0;
    hand_open = false;
    hand_time = 1000;
    hook_right = 0.0;
    
    shoulder_omega = 0;
    winch_omega = 0;
    inside_elbow_omega = 0;
    
    shoulder_compensation = 0;
    
     //TODO: figure out a better way to have things reset to their initial values
    armFunction = armAutonomousControl;
    
    score_mode = false;

    suppress_arm = true;
    
    initCamera();
    
    // char * log_file_name = "autonomous_log_0.txt";
    
    // for(int i = 1; access(log_file_name, F_OK) != -1; i++)
    // {
    //     sprintf(log_file_name, "autonomus_log_%d.txt", i);
    // }
    // log_file = fopen(log_file_name, "w");
    
    //waitForStart(); //NOTE: needs to be called in java until IMU code is ported
    zeroDriveSensors();
    //enableKillerAI();
    
    start_time = time;
    
    imu_orientation_offsets = (v3f){0, 0, 0};
    updateRobot();
    imu_orientation_offsets = (v3f){pimu_values->orientation.x, pimu_values->orientation.y, pimu_values->orientation.z};
    current_time = time;
    //Config
    //hopper down
    #define colorAdjustedAngle(a) ((current_color) ? (a) : -(a))
    #define blocks_in_hopper 1
    interruptable
    {
        for(int i = 0; i < 2; i++)
        {
            float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                              -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
            target_arm_theta = shoulder_theta-asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta));
            target_arm_y = shoulder_axis_to_end*cos(target_arm_theta-vertical_arm_theta);
            target_shoulder_theta = shoulder_theta;
            target_inside_elbow_theta = inside_elbow_theta;
                
            autonomousUpdate();
        }
        
        #if 0 // test driveOnCourseIn
        target_shoulder_theta = shoulder_theta;
        target_inside_elbow_theta = inside_elbow_theta;
        driveOnCourseIn(60, -0.8, colorAdjustedAngle(0));//Drive 120 in at 45 degrees, relative to the driver box
            
        waitForEnd();
        #endif

        if(current_color)
        {
            driveOnCourseIn(17, -0.8, 0);
        }
        else
        {
            driveOnCourseIn(10, -0.8, 0);
        }
        turnRelDeg(colorAdjustedAngle(45), 1.0);
        
        //Deploy Robot
        //Wait time delay
        //Turn on intake
        //intake = 1;
        
        //Drive to goal
        
        if(current_color)
        {
            driveOnCourseIn(46, -0.8, colorAdjustedAngle(45));//Drive 120 in at 45 degrees, relative to the driver box
        }
        else
        {
            driveOnCourseIn(41, -0.8, colorAdjustedAngle(45));//Drive 120 in at 45 degrees, relative to the driver box
        }
        
        //Once 5 blocks are reached, reverse intake direction
        // if(blocks_in_hopper >= 5)
        //     intake = -1;
        
        //intake = 0;
        
        //Turn and align with beacon
        #if 0
        score_mode = false;
        arm_stage = arm_retracting;
        
        while(arm_stage != arm_idle)
        {
            autonomousUpdate();
        }
        
        target_shoulder_theta = 1.4;
        target_inside_elbow_theta = 4.0*pi/8.0;
        while(!armIsAtTarget(0.25, 0.25))
        {
            autonomousUpdate();
        }
        #endif
        
        turnRelDeg(colorAdjustedAngle(45), 1.0);
        
        wait(0.25);
        // bool color = getBeaconColor(); //TODO: NOTE: this function crashes auto
        // setIntakeIn();
        //wait(0.75);
        //Drive forward a bit
        //driveDistIn(1, -0.6);
        
        //Score climbers
        if(current_color)
        {
            driveDistIn(3, -0.8);
        }
        else
        {
            driveDistIn(15, -0.8);
        }
        hook_right = 1.0;
        wait(3.0);
        hook_right = 0.0;
        
        driveDistIn(10, 0.8);
        
        //Push button
        #if 0
        hook_right = 1.0;
        //TODO: Function for setting the intake to any angle
        // if(color == current_color)
        // {
        //     turnRelDeg(5, 0.8);
        //     driveDistIn(10, -0.8);//Pushing button
        //     wait(0.5);
        //     driveDistIn(10, 0.8);//Pushing button
        //     turnRelDeg(5, -0.8);
        // }
        // else
        // {
        //     turnRelDeg(5, -0.8);
        //     driveDistIn(10, -0.8);//Pushing button
        //     wait(0.5);
        //     driveDistIn(10, 0.8);//Pushing button
        //     turnRelDeg(5, 0.8);
        // }
        //Drive out of parking zone
        //driveDistIn(11, 0.5);
        #endif
        //Turn and park on nearest low mountain
        //turnRelDeg(colorAdjustedAngle(45), -0.8);
        
        #if 1 //enable arm
        suppress_arm = false;
        
        intake_out = true;
        intake_time = 0.0;
        wait(0.5);
        
        // intake = -1;
        // driveOnCourseIn(5, 0.8, colorAdjustedAngle(45));
        // driveOnCourseIn(5, -0.8, colorAdjustedAngle(45));
        // intake = 0;
        
        target_shoulder_theta = 1.4;
        target_inside_elbow_theta = 9.0*pi/8.0;
        while(!armIsAtTarget(0.25, 0.25))
        {
            autonomousUpdate();
        }
        wait(0.5);
        
        target_shoulder_theta = 1.5;
        target_inside_elbow_theta = 4.0;
        while(!armIsAtTarget(0.1, 0.1))
        {
            autonomousUpdate();
        }
        
        wait(0.2);
        
        score_mode = false;
        armFunction = armToScoreMode;
        
        float arm_timer = 0;
        while(armFunction != armAutonomousControl)
        {
            arm_timer += dt;
            if(arm_timer > 3)
            {
                target_shoulder_theta = 2.0;
                target_inside_elbow_theta = 4.43;
                wait(0.5);
                intake = -1;
                driveDistIn(10, -0.1);
                intake = 0;
                armFunction = armAutonomousControl;
                arm_timer = 0;
            }
            autonomousUpdate();
        }
        wait(0.2);
        
        turnRelDeg(180, 1.0);
        
        intake = 1;
        driveDistIn(15, -0.8);
        driveDistIn(15, 0.8);
        intake = 0;
        turnRelDeg(45, 1.0);
        
        #endif
        
        // driveOnCourseIn(33, 0.8, colorAdjustedAngle(45));
        // // setIntakeIn();
        // //Arm to partial extension for teleop
        // //turnRelDeg(colorAdjustedAngle(-90), -0.8);
        // driveOnCourseIn(30, 0.8, colorAdjustedAngle(131));
        // while(fabs(imu_tilt) < 20)
        // {
        //     left_drive = 0.7;
        //     right_drive = 0.7;
        //     autonomousUpdate();
        // }
        // score_hook = 1.0;
        //Might need some sort of traction type thing, or use the imu until we're over the first churro/stuck on it
        //Flash LEDs to show auto is done
        waitForEnd();
    }
    if(log_file) fclose(log_file);
    
    cleanupCamera();
}
 
