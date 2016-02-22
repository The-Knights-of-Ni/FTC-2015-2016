#include "autonomous.h" //NOTE: needs customAutonomousUpdate must be declared before including this
#include "vision.h"

#include "jni_functions.h"

//stuff that need to be constantly updated in the background but is not intensive enough to deserve a seperate thread
void Mk4AutonomousUpdate()
{
    dt = time-current_time;
    current_time = time;

    //TODO: make this sensor filter stuff a function in arm.h

    updateArmSensors();
    
    shoulder = 0;
    winch = 0;
    armToJointTarget(); //TODO; might want to use armToPreset
    doIntake();
    doHand();
    
    //clamp the integral factors to stop integral build up
    shoulder_compensation = clamp(shoulder_compensation, -1.0, 1.0);
    winch_compensation = clamp(winch_compensation, -1.0, 1.0);
    
    shoulder = clamp(shoulder, -1.0, 1.0);
    winch = clamp(winch, -1.0, 1.0);
    left_drive = clamp(left_drive, -1.0, 1.0);
    right_drive = clamp(right_drive, -1.0, 1.0);
    intake = clamp(intake, -1.0, 1.0);
    
    hand = clamp(hand, 0.0, 1.0);
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
        "import android.hardware.Camera;\n");
    
    //TODO: shortcut for defining and declaring motors, servos, etc.
    jni_variables_string = (
        "/* Start Motor Definitions */\n"
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
        "shoulder.setDirection(DcMotor.Direction.REVERSE);\n"
        "//winch.setDirection(DcMotor.Direction.REVERSE);\n"
        "\n"
        "hand = hardwareMap.servo.get(\"hand\");\n"
        "wrist = hardwareMap.servo.get(\"wrist\");\n"
        "hook_left = hardwareMap.servo.get(\"hook_left\");\n"
        "hook_right = hardwareMap.servo.get(\"hook_right\");\n"
        "hook_left.setDirection(Servo.Direction.REVERSE);\n"
        "intake_tilt = hardwareMap.servo.get(\"intake_tilt\");\n"
        "intake_tilt.setDirection(Servo.Direction.REVERSE);"
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
                              "camera.addCallbackBuffer(camera_buffer);\n");
    
    ptime = jniDoubleIn("return time;");
    pright_drive_encoder = jniIntIn("return 0;//right_drive.getCurrentPosition();");
    pleft_drive_encoder = jniIntIn("return 0;//left_drive.getCurrentPosition();");
    pwinch_encoder = jniIntIn("return winch.getCurrentPosition();");
    pshoulder_encoder = jniIntIn("return shoulder.getCurrentPosition();");
    pelbow_potentiometer = jniIntIn("return dim.getAnalogInputValue(elbow_potentiometer_port);");
    pshoulder_potentiometer = jniIntIn("return dim.getAnalogInputValue(shoulder_potentiometer_port);");
    
    pimu_values = jniStructIn(
        imu_state,
        "if(imu.checkForUpdate()) {\n"
        "    return {imu.eul_x, imu.eul_y, imu.eul_z, imu.vel_x, imu.vel_y, imu.vel_z};\n"
        "}\n");
    
    int * pcurrent_color;
    #define current_color (*pcurrent_color)
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
    jniOut("telemetry.addData(\"beacon right:\", (", pbeacon_right," == 1 ? \"red\" : \"blue\"));");
    short * pimu_heading = &imu_heading;
    jniOut("telemetry.addData(\"heading:\", ", pimu_heading, ");");
    
    jniGenerate();
    
    
    initCamera();
    
    //waitForStart(); //NOTE: needs to be called in java until IMU code is ported
    zeroDriveSensors();
    
    current_time = 0;
    //Config
    //hopper down
    #define colorAdjustedAngle(a) (currentColor ? (a) : -(a))
    #define blocks_in_hopper 1
    interruptable
    {
        driveOnCourseIn(24, -0.8, 45);
        driveOnCourseIn(24, 0.8, 45);
        for ever
        {
            autonomousUpdate();
        }
        
        //Deploy Robot
        //Wait time delay
        //Turn on intake
        intake = 1;
        
        //Drive to goal
        driveOnCourseIn(120, -0.8, 45);//Drive 120 in at 45 degrees, relative to the driver box
        //Once 5 blocks are reached, reverse intake direction
        if(blocks_in_hopper >= 5)
            intake = -1;
        //Turn and align with beacon
        turnRelDeg(45, 0.8);
        //Drive forward a bit
        driveDistIn(10, -0.8);
        //Score climbers
        setIntakeIn();
        driveDistIn(5, -0.5);
        setIntakeOut();//TODO: Make this less, we just need to tap it on the top to release the climbers
        //Push button
        driveDistIn(5, 0.5);
        //TODO: Function for setting the intake to any angle
        bool color = getBeaconColor();
        if(color == current_color)
            setIntakeOut();//The right side is the right color
        else
            setIntakeOut();//The left side is the right color
        driveDistIn(1, -0.5);//Pushing button
        //Drive out of parking zone
        driveDistIn(11, 0.5);
        //Turn and park on nearest low mountain
        turnRelDeg(45, -0.8);
        driveDistIn(60, 0.8);
        setIntakeIn();
        //Arm to partial extension for teleop
        turnRelDeg(90, -0.8);
        driveDistIn(40, 0.8);
        //Might need some sort of traction type thing, or use the imu until we're over the first churro/stuck on it
        //Flash LEDs to show auto is done
        for ever
        {
            autonomousUpdate();
        }
    }
    cleanupCamera();
}
