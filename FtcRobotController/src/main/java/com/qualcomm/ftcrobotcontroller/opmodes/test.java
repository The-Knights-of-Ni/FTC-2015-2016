/*
WARNING: this is a generated file
changes made this file are not permanent
*/
package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.ftccommon.DbgLog;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import java.nio.ByteBuffer;
import android.hardware.Camera;
import android.graphics.ImageFormat;
import android.os.Environment;


public class test extends LinearOpMode {
public static byte[] robot_state;
int rsid_current = 0;
public test()
{
    DbgLog.error("opmode constructor");
    robot_state = new byte[52];

camera = FtcRobotControllerActivity.camera_preview.camera;
Camera.Parameters parameters = camera.getParameters();
Camera.Size camera_size = parameters.getPreviewSize();
camera_w = camera_size.width;
camera_h = camera_size.height;

camera_buffer = new byte[camera_w*camera_h*4];
camera_preview_callback = new CameraPreviewCallback();

camera.setPreviewCallbackWithBuffer(camera_preview_callback);
camera.addCallbackBuffer(camera_buffer);
parameters.setPreviewFormat(ImageFormat.NV21);
parameters.setExposureCompensation(-2);
parameters.setWhiteBalance(Camera.Parameters.WHITE_BALANCE_FLUORESCENT);
parameters.set("iso", "ISO100");
parameters.set("max-exposure-time", 2000000);
parameters.set("min-exposure-time", 2000000);
parameters.set("contrast", 2);
DbgLog.error("Camera parameters: "+parameters.flatten());
camera.setParameters(parameters);
}


public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.
{
    return ByteBuffer.wrap(joystick, 42, 4).getInt();
}

Camera camera = null;
int camera_w = 0;
int camera_h = 0;
CameraPreviewCallback camera_preview_callback;

byte[] camera_buffer = null;

class CameraPreviewCallback implements Camera.PreviewCallback
{
    
    
    CameraPreviewCallback(){}
    public void onPreviewFrame(byte[] data, Camera camera)
    {
        camera.addCallbackBuffer(camera_buffer);
    }
}
public boolean isExternalStorageWritable()
{
    String state = Environment.getExternalStorageState();
    if (Environment.MEDIA_MOUNTED.equals(state)) {
        return true;
    }
    return false;
}

public void setShort(int index, short a)
{
    rsid_current = index;
    ByteBuffer.wrap(robot_state, rsid_current, 2).order(ByteOrder.nativeOrder()).putShort(a);
    rsid_current = index+2;
}
public void setRelative(short a)
{
    ByteBuffer.wrap(robot_state, rsid_current, 2).order(ByteOrder.nativeOrder()).putShort(a);
    rsid_current += 2;
}
public short getShort(int index)
{
    rsid_current = index+2;
    return ByteBuffer.wrap(robot_state, index, 2).order(ByteOrder.nativeOrder()).getShort();
}
public short getRelativeShort()
{
    short out = ByteBuffer.wrap(robot_state, rsid_current, 2).order(ByteOrder.nativeOrder()).getShort();
    rsid_current += 2;
    return out;
}

public void setInt(int index, int a)
{
    rsid_current = index;
    ByteBuffer.wrap(robot_state, rsid_current, 4).order(ByteOrder.nativeOrder()).putInt(a);
    rsid_current = index+4;
}
public void setRelative(int a)
{
    ByteBuffer.wrap(robot_state, rsid_current, 4).order(ByteOrder.nativeOrder()).putInt(a);
    rsid_current += 4;
}
public int getInt(int index)
{
    rsid_current = index+4;
    return ByteBuffer.wrap(robot_state, index, 4).order(ByteOrder.nativeOrder()).getInt();
}
public int getRelativeInt()
{
    int out = ByteBuffer.wrap(robot_state, rsid_current, 4).order(ByteOrder.nativeOrder()).getInt();
    rsid_current += 4;
    return out;
}

public void setLong(int index, long a)
{
    rsid_current = index;
    ByteBuffer.wrap(robot_state, rsid_current, 8).order(ByteOrder.nativeOrder()).putLong(a);
    rsid_current = index+8;
}
public void setRelative(long a)
{
    ByteBuffer.wrap(robot_state, rsid_current, 8).order(ByteOrder.nativeOrder()).putLong(a);
    rsid_current += 8;
}
public long getLong(int index)
{
    rsid_current = index+8;
    return ByteBuffer.wrap(robot_state, index, 8).order(ByteOrder.nativeOrder()).getLong();
}
public long getRelativeLong()
{
    long out = ByteBuffer.wrap(robot_state, rsid_current, 8).order(ByteOrder.nativeOrder()).getLong();
    rsid_current += 8;
    return out;
}

public void setFloat(int index, float a)
{
    rsid_current = index;
    ByteBuffer.wrap(robot_state, rsid_current, 4).order(ByteOrder.nativeOrder()).putFloat(a);
    rsid_current = index+4;
}
public void setRelative(float a)
{
    ByteBuffer.wrap(robot_state, rsid_current, 4).order(ByteOrder.nativeOrder()).putFloat(a);
    rsid_current += 4;
}
public float getFloat(int index)
{
    rsid_current = index+4;
    return ByteBuffer.wrap(robot_state, index, 4).order(ByteOrder.nativeOrder()).getFloat();
}
public float getRelativeFloat()
{
    float out = ByteBuffer.wrap(robot_state, rsid_current, 4).order(ByteOrder.nativeOrder()).getFloat();
    rsid_current += 4;
    return out;
}

public void setDouble(int index, double a)
{
    rsid_current = index;
    ByteBuffer.wrap(robot_state, rsid_current, 8).order(ByteOrder.nativeOrder()).putDouble(a);
    rsid_current = index+8;
}
public void setRelative(double a)
{
    ByteBuffer.wrap(robot_state, rsid_current, 8).order(ByteOrder.nativeOrder()).putDouble(a);
    rsid_current += 8;
}
public double getDouble(int index)
{
    rsid_current = index+8;
    return ByteBuffer.wrap(robot_state, index, 8).order(ByteOrder.nativeOrder()).getDouble();
}
public double getRelativeDouble()
{
    double out = ByteBuffer.wrap(robot_state, rsid_current, 8).order(ByteOrder.nativeOrder()).getDouble();
    rsid_current += 8;
    return out;
}


void robotStateOut()
{
rsid_current = 0;
telemetry.addData("left drive",getFloat(36));

telemetry.addData("right drive",getFloat(40));

telemetry.addData("time",getFloat(44));

telemetry.addData("beacon",getInt(48));


}

void robotStateIn()
{
{
setDouble(0, time);

rsid_current = 8;
}
{
rsid_current = 8;
int gamepad1_buttons = 0;
try
{
    gamepad1_buttons = updateButtons(gamepad1.toByteArray());
}
catch (RobotCoreException e)
{
    e.printStackTrace();
}
telemetry.addData("gamepad1_buttons", String.format("%d", gamepad1_buttons));
setRelative(gamepad1.left_stick_x);
setRelative( gamepad1.left_stick_y);
setRelative(gamepad1.right_stick_x);
setRelative( gamepad1.right_stick_y);
setRelative(gamepad1.left_trigger);
setRelative( gamepad1.right_trigger);
setRelative( gamepad1_buttons);
;
}

}

native void main();

static
{
    System.loadLibrary("native_robot");
}

@Override public void runOpMode() throws InterruptedException
{
    main();
}
}
