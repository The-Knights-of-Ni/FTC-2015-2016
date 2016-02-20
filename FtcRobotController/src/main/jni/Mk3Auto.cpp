#include "autonomous.h" //NOTE: needs customAutonomousUpdate must be declared before including this
#include "vision.h"

#include "jni_functions.h"

//stuff that need to be constantly updated in the background but is not intensive enough to deserve a seperate thread
void Mk3AutonomousUpdate()
{
    dt = time-current_time;
    current_time = time;
    
    //TODO: make this sensor filter stuff a function in arm.h
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
    
    shoulder_theta = new_shoulder_theta;
    inside_elbow_theta = new_inside_elbow_theta;
    winch_theta = new_winch_theta;
    
    shoulder = 0;
    winch = 0;
    armToJointTarget();
    
    shoulder = clamp(shoulder, -1.0, 1.0);
    winch = clamp(winch, -1.0, 1.0);
    left_drive = clamp(left_drive, -1.0, 1.0);
    right_drive = clamp(right_drive, -1.0, 1.0);
    intake = clamp(intake, -1.0, 1.0);
    
    hand = clamp(hand, 0.0, 1.0);
}

#ifndef GENERATE
#undef jniMain
#define jniMain Java_com_qualcomm_ftcrobotcontroller_opmodes_Mk3Auto_main
#endif

//#define update (if(updateRobot() == 0) exit(EXIT_SUCCESS););
#define currentColor 0
#define visionColor 0 //Right side of the beacon
#define slide_position_right 1
#define slide_position_left 0
#define slide_position_safe 0.5

extern "C"
void jniMain(JNIEnv * _env, jobject _self)
{
    //NOTE: DON'T FORGET THESE //TODO: make it so you don't need these
    env = _env;
    self = _self;    
    initJNI();
    
    customAutonomousUpdate = Mk3AutonomousUpdate;
    
    jni_import_string = (
        "import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;\n"
        "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n"
        "import com.qualcomm.robotcore.exception.RobotCoreException;\n"
        "import com.qualcomm.robotcore.hardware.DcMotor;\n"
        "import com.qualcomm.robotcore.hardware.DcMotorController;\n"
        "import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;\n"
        "import com.qualcomm.robotcore.hardware.I2cDevice;\n"
        "import com.qualcomm.robotcore.hardware.Servo;\n"
        "import android.hardware.Camera;\n");

    //TODO: shortcut for defining and declaring motors, servos, etc.
    jni_variables_string = (
        "/* Start Motor Definitions */\n"
        "DeviceInterfaceModule dim;\n"
        "IMU imu;"
        "int elbow_potentiometer_port = 7;\n"
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
    
    jni_run_opmode_string = (
        "dim = hardwareMap.deviceInterfaceModule.get(\"dim\");\n"
        "I2cDevice imu_i2c_device = hardwareMap.i2cDevice.get(\"imu\");\n"
        "imu = new IMU(imu_i2c_device);\n"
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
        "hook_left.setDirection(Servo.Direction.REVERSE);"

        "while (!FtcRobotControllerActivity.aligned || (!FtcRobotControllerActivity.red && !FtcRobotControllerActivity.blue))\n"
        "{\n"
        "    telemetry.addData(\"unchecked boxes\", \"fix it\");\n"
        "    waitForNextHardwareCycle();\n"
        "}\n"
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
        "}\n"
        "\n"
        "public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.\n"
        "{\n"
        "    ByteBuffer stick = ByteBuffer.allocate(45);\n"
        "    stick.put(joystick);\n"
        "    return stick.getInt(40);//Offset value\n"
        "}\n");
    
    jni_constructor_string = ("camera = FtcRobotControllerActivity.camera;\n"
                              "Camera.Parameters parameters = camera.getParameters();\n"
                              "Camera.Size camera_size = parameters.getPreviewSize();\n"
                              "camera_w = camera_size.width;\n"
                              "camera_h = camera_size.height;\n"
                              "\n"
                              "camera_buffer = new byte[camera_w*camera_h*4];\n"
                              "camera_preview_callback = new CameraPreviewCallback();\n"
                              "\n"
                              "camera.setPreviewCallbackWithBuffer(camera_preview_callback);\n"
                              "camera.addCallbackBuffer(camera_buffer);\n");
    
    ptime = jniDoubleIn("return time;");
    pright_drive_encoder = jniIntIn("return right_drive.getCurrentPosition();");
    pleft_drive_encoder = jniIntIn("return left_drive.getCurrentPosition();");
    pwinch_encoder = jniIntIn("return winch.getCurrentPosition();");
    pshoulder_encoder = jniIntIn("return shoulder.getCurrentPosition();");
    pelbow_potentiometer = jniIntIn("return dim.getAnalogInputValue(elbow_potentiometer_port);");
    
    pimu_values = jniStructIn(
        imu_state,
        "if(imu.checkForUpdate()) {\n"
        "    return {imu.eul_x, imu.eul_y, imu.eul_z, imu.vel_x, imu.vel_y, imu.vel_z};\n"
        "}\n");
    
    int * current_color = jniIntIn("return (FtcRobotControllerActivity.red ? 1 : 0);");
    
    jniOut("left_drive.setPower(", pleft_drive, ");");
    jniOut("right_drive.setPower(", pright_drive, ");");
    jniOut("winch.setPower(", pwinch, ");");
    jniOut("shoulder.setPower(", pshoulder, ");");
    jniOut("intake.setPower(", pintake, ");");
    
    jniOut("hand.setPosition(", phand,");");
    jniOut("hook_left.setPosition(", phook_left,");");
    jniOut("hook_right.setPosition(", phook_right,");");
    
    jniOut("telemetry.addData(\"Indicator:\", ", pindicator, ");");
    jniOut("telemetry.addData(\"beacon right:\", (", pbeacon_right," == 1 ? \"red\" : \"blue\"));");
    float * pimu_heading = &imu_heading;
    jniOut("telemetry.addData(\"heading:\", ", pimu_heading, ");");
    
    jniGenerate();
    
    waitForStart();
    initCamera();
    
    //waitForStart(); //needs to be called in java until IMU code is ported
    
    current_time = 0;
    //Config
    //hopper down
    #define colorAdjustedAngle(a) (currentColor ? a : -a)
    
    interruptable
    {
        // for ever
        // {
        //     beacon_right = getBeaconColor();
        //     updateRobot();
        // }
        
        //target_shoulder_theta = pi*140/180; don't think I need it to change pos
        /*target_inside_elbow_theta = pi*30/180;
        for ever
        {
            autonomousUpdate();
        }*/
        wait(1);
        //intake = 1;
        // slide moves at 3.25 in/sec
        /*slide = 1.0;
        wait(0.5);
        slide = 0.5;
        for ever
        {
            autonomousUpdate();
        }*/
        autonomousUpdate();
        //driveOnCourseIn(-10, 0.8, 45);
        //turnRelDeg(45, 0.8);
        //#if 0
        driveOnCourseIn(80, -0.8, imu_heading);
        #if 0
        intake = 0;
        (colorAdjustedAngle(45), 0.8);
        //vision
        if (visionColor == currentColor)
        {
            slide = slide_position_right;
        }
        else
            slide = slide_position_left;
        wait(1);
        driveDistIn(-5, 0.4);
        //score climbers, ideally without turning
        target_arm_theta = 150;
        target_inside_elbow_theta = 180;
        wait(1);
        driveDistIn(24, 0.8);
        turnRelDeg(colorAdjustedAngle(45), 0.8);
        driveDistIn(24);
        turnRelDeg(colorAdjustedAngle(90), 0.8);
        //driveDistIn(60);
        #endif
        //drive to nearest mountain, park low

        for ever
        {
            autonomousUpdate();
        }
    }
    cleanupCamera();
}
