#include "jni_functions.h"
#include "button.h"
#include "drive.h"

#ifndef GENERATE
#undef JNI_main
#define jniMain Java_com_qualcomm_ftcrobotcontroller_opmodes_test_main
#endif

extern "C" void jniMain(JNIEnv * _env, jobject _self)
{
    env = _env;
    self = _self;
    initJNI();
    
    jni_import_string = ("import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;\n"
                         "import com.qualcomm.robotcore.exception.RobotCoreException;\n"
                         "import com.qualcomm.robotcore.hardware.DcMotor;\n"
                         "import com.qualcomm.robotcore.hardware.DcMotorController;\n"
                         "import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;\n"
                         "import com.qualcomm.robotcore.hardware.Servo;\n"
                         "import java.nio.ByteBuffer;\n");
    #if 0
    //TODO: shortcut for defining and declaring motors, servos, etc.
    jni_variables_string = ("/* Start Motor Definitions */\n"
                            "DeviceInterfaceModule dim;\n"
                            "\n"
                            "DcMotor left_drive;\n"
                            "DcMotor right_drive;\n"
                            "DcMotor shoulder;\n"
                            "DcMotor elbow;\n"
                            "DcMotor intake;\n"
                            "\n"
                            "Servo hand_servo;\n"
                            "Servo slide_servo;\n"
                            "Servo hook_left_servo;\n"
                            "Servo hook_right_servo;\n"
                            "/* End Motor Definitions */");
    
    jni_run_opmode_string = ("int elbow_potentiometer_port = 7;\n"
                             "dim = hardwareMap.deviceInterfaceModule.get(\"dim\");\n"
                             "left_drive  = hardwareMap.dcMotor.get(\"leftd\");\n"
                             "right_drive = hardwareMap.dcMotor.get(\"rightd\");\n"
                             "shoulder    = hardwareMap.dcMotor.get(\"shoulder\");\n"
                             "elbow       = hardwareMap.dcMotor.get(\"winch\");\n"
                             "intake      = hardwareMap.dcMotor.get(\"intake\");\n"
                             "right_drive.setDirection(DcMotor.Direction.REVERSE);\n"
                             "left_drive.setDirection(DcMotor.Direction.REVERSE);\n"
                             "shoulder.setDirection(DcMotor.Direction.REVERSE);\n"
                             "shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);\n"
                             "waitOneFullHardwareCycle();\n"
                             "intake.setDirection(DcMotor.Direction.REVERSE);\n"
                             "shoulder.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);\n"
                             "elbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);\n"
                             "waitOneFullHardwareCycle();\n"
                             "elbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);\n"
                             "waitOneFullHardwareCycle();\n"
                             "//shoulder.setDirection(DcMotor.Direction.REVERSE);\n"
                             "//elbow.setDirection(DcMotor.Direction.REVERSE);\n"
                             "\n"
                             "hand_servo = hardwareMap.servo.get(\"hand\");\n"
                             "slide_servo = hardwareMap.servo.get(\"slide\");\n"
                             "hook_left_servo = hardwareMap.servo.get(\"hook_left\");\n"
                             "hook_right_servo = hardwareMap.servo.get(\"hook_right\");\n"
                             "hook_left_servo.setDirection(Servo.Direction.REVERSE);");
    #endif
    
    jni_misc_string = (
        "public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.\n"
        "{\n"
        "    return ByteBuffer.wrap(joystick, 42, 4).getInt();\n"
        "}\n");
    
    // pleft_drive_encoder = jniIntIn("return left_drive.getCurrentPosition();");
    // pright_drive_encoder = jniIntIn("return right_drive.getCurrentPosition();");
    
    ptime = jniDoubleIn("return time;");
    
    //NOTE: this only works for tightly packed structs
    pgamepad1 = jniStructIn(
        gamepad,
        "int gamepad1_buttons = 0;\n"
        "try\n"
        "{\n"
        "    //TODO: toByteArray() just copies all of the values into a byte buffer anyway\n"
        "    //      make custom function so this won't break again if they update\n"
        "    gamepad1_buttons = updateButtons(gamepad1.toByteArray());\n"
        "}\n"
        "catch (RobotCoreException e)\n"
        "{\n"
        "    e.printStackTrace();\n"
        "}\n"
        "return {gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.left_trigger, gamepad1.right_trigger, gamepad1_buttons};");
    
    jniOut("telemetry.addData(\"left drive\",", pleft_drive, ");\n");
    jniOut("telemetry.addData(\"right drive\",", pright_drive, ");\n");
    float * pdt;
    jniOut("telemetry.addData(\"time\",", pdt, ");\n");
    jniGenerate();
    
    Button pad1 = {};
    
    waitForStart();
    
    interruptable for ever
    {
        dt = time-current_time;
        *pdt = dt;
        current_time = time;
        
        left_drive = gamepad1.joystick1.y-gamepad1.joystick1.x;
        right_drive = gamepad1.joystick1.y+gamepad1.joystick1.x;
        
        pad1.updateButtons(gamepad1);
        updateRobot();
    }
}
