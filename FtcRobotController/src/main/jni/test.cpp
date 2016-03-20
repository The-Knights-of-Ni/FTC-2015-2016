#include "jni_functions.h"
#include "button.h"
#include "drive.h"
#include "logging.h"

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
                         "import java.nio.ByteBuffer;\n"
                         logging_jni_import_string);
    
    jni_misc_string = (
        "public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.\n"
        "{\n"
        "    return ByteBuffer.wrap(joystick, 42, 4).getInt();\n"
        "}\n"
        logging_jni_misc_string);
    
    // pleft_drive_encoder = jniIntIn("return left_drive.getCurrentPosition();");
    // pright_drive_encoder = jniIntIn("return right_drive.getCurrentPosition();");
    
    ptime = jniDoubleIn("return time;");
    
    //NOTE: this only works for tightly packed structs
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
    
    jniOut("telemetry.addData(\"left drive\",", pleft_drive, ");\n");
    jniOut("telemetry.addData(\"right drive\",", pright_drive, ");\n");
    float * pdt;
    jniOut("telemetry.addData(\"time\",", pdt, ");\n");
    int * file_pointer;
    jniOut("telemetry.addData(\"file pointer\",", file_pointer, ");\n");
    
    jniGenerate();
    
    initLogfile();
    
    //*file_pointer = (int) logfile;
    
    Button pad1 = {};
    
    waitForStart();
    
    interruptable for ever
    {
        dt = time-current_time;
        *pdt = dt;
        current_time = time;
        
        log("dt: %f\n", dt);
        
        left_drive = gamepad1.joystick1.y-gamepad1.joystick1.x;
        right_drive = gamepad1.joystick1.y+gamepad1.joystick1.x;
        
        pad1.updateButtons(gamepad1);
        updateRobot();
    }
    
    closeLogfile();
}
