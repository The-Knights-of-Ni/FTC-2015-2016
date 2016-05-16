#include "jni_functions.h"
#include "button.h"
#include "drive.h"
#include "logging.h"
#include "vision.h"

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
                         "import android.hardware.Camera;\n"
                         "import android.graphics.ImageFormat;\n"
                         logging_jni_import_string);
    
    jni_misc_string = (
        "public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.\n"
        "{\n"
        "    return ByteBuffer.wrap(joystick, 42, 4).getInt();\n"
        "}\n"
        "\n"
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
        logging_jni_misc_string);
    
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
                              "parameters.setExposureCompensation(-2);\n"
                              "parameters.setWhiteBalance(Camera.Parameters.WHITE_BALANCE_FLUORESCENT);\n"
                              "parameters.set(\"iso\", \"ISO100\");\n"
                              "parameters.set(\"max-exposure-time\", 2000000);\n"
                              "parameters.set(\"min-exposure-time\", 2000000);\n"
                              "parameters.set(\"contrast\", 2);\n"
                              "DbgLog.error(\"Camera parameters: \"+parameters.flatten());\n"
                              "camera.setParameters(parameters);");
    
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
    int * beacon;
    jniOut("telemetry.addData(\"beacon\",", beacon, ");\n");
    
    jniGenerate();
    
    initLogfile();
        
    Button pad1 = {};
    
    initCamera();
    
    waitForStart();
    
    interruptable for ever
    {
        dt = time-current_time;
        *pdt = dt;
        current_time = time;
        
        *beacon = getBeaconColor();
        
        log("beacon: %d  dt: %f\n", *beacon, dt);
        
        left_drive = gamepad1.joystick1.y-gamepad1.joystick1.x;
        right_drive = gamepad1.joystick1.y+gamepad1.joystick1.x;
        
        pad1.updateButtons(gamepad1);
        updateRobot();
    }
    
    cleanupCamera();
    closeLogfile();
}
