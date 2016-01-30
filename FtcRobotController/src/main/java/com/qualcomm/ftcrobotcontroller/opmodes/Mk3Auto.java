/*
WARNING: this is a generated file
changes made this file are not permanent
*/
package com.qualcomm.ftcrobotcontroller.opmodes;


import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import android.hardware.Camera;


public class Mk3Auto extends LinearOpMode {
byte[] robot_state;
int rsid_current = 0;
public Mk3Auto()
{
    robot_state = new byte[100];

camera = FtcRobotControllerActivity.camera;
Camera.Parameters parameters = camera.getParameters();
Camera.Size camera_size = parameters.getPreviewSize();
camera_w = camera_size.width;
camera_h = camera_size.height;

camera_buffer = new byte[camera_w*camera_h*4];
camera_preview_callback = new CameraPreviewCallback();

camera.setPreviewCallbackWithBuffer(camera_preview_callback);
camera.addCallbackBuffer(camera_buffer);

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

public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.
{
    ByteBuffer stick = ByteBuffer.allocate(45);
    stick.put(joystick);
    return stick.getInt(40);//Offset value
}

public void setInt(int a)
{
    ByteBuffer.wrap(robot_state, rsid_current, rsid_current+4).order(ByteOrder.nativeOrder()).putInt(a);
    rsid_current += 4;
}
public void set(int a)
{
    setInt(a);
}
public int getInt()
{
    int out = ByteBuffer.wrap(robot_state, rsid_current, rsid_current+4).order(ByteOrder.nativeOrder()).getInt();
    rsid_current += 4;
    return out;}
public int getExistingInt(int rsid_existing)
{
    return ByteBuffer.wrap(robot_state, rsid_existing, rsid_existing+4).order(ByteOrder.nativeOrder()).getInt();
}

public void setLong(long a)
{
    ByteBuffer.wrap(robot_state, rsid_current, rsid_current+8).order(ByteOrder.nativeOrder()).putLong(a);
    rsid_current += 8;
}
public void set(long a)
{
    setLong(a);
}
public long getLong()
{
    long out = ByteBuffer.wrap(robot_state, rsid_current, rsid_current+8).order(ByteOrder.nativeOrder()).getLong();
    rsid_current += 8;
    return out;}
public long getExistingLong(int rsid_existing)
{
    return ByteBuffer.wrap(robot_state, rsid_existing, rsid_existing+8).order(ByteOrder.nativeOrder()).getLong();
}

public void setFloat(float a)
{
    ByteBuffer.wrap(robot_state, rsid_current, rsid_current+4).order(ByteOrder.nativeOrder()).putFloat(a);
    rsid_current += 4;
}
public void set(float a)
{
    setFloat(a);
}
public float getFloat()
{
    float out = ByteBuffer.wrap(robot_state, rsid_current, rsid_current+4).order(ByteOrder.nativeOrder()).getFloat();
    rsid_current += 4;
    return out;}
public float getExistingFloat(int rsid_existing)
{
    return ByteBuffer.wrap(robot_state, rsid_existing, rsid_existing+4).order(ByteOrder.nativeOrder()).getFloat();
}

public void setDouble(double a)
{
    ByteBuffer.wrap(robot_state, rsid_current, rsid_current+8).order(ByteOrder.nativeOrder()).putDouble(a);
    rsid_current += 8;
}
public void set(double a)
{
    setDouble(a);
}
public double getDouble()
{
    double out = ByteBuffer.wrap(robot_state, rsid_current, rsid_current+8).order(ByteOrder.nativeOrder()).getDouble();
    rsid_current += 8;
    return out;}
public double getExistingDouble(int rsid_existing)
{
    return ByteBuffer.wrap(robot_state, rsid_existing, rsid_existing+8).order(ByteOrder.nativeOrder()).getDouble();
}


void robotStateIn()
{
rsid_current = 0;
{
setDouble(time);

rsid_current = 8;
}
{
setInt(right_drive.getCurrentPosition());

rsid_current = 12;
}
{
setInt(left_drive.getCurrentPosition());

rsid_current = 16;
}
{
setInt(winch.getCurrentPosition());

rsid_current = 20;
}
{
setInt(shoulder.getCurrentPosition());

rsid_current = 24;
}
{
setInt(dim.getAnalogInputValue(elbow_potentiometer_port));

rsid_current = 28;
}
{
if(imu.checkForUpdate()) {
    set(imu.eul_x);
set( imu.eul_y);
set( imu.eul_z);
set( imu.vel_x);
set( imu.vel_y);
set( imu.vel_z);
;
}

}
{
setInt((FtcRobotControllerActivity.red ? 1 : 0));

rsid_current = 56;
}

}

void robotStateOut()
{
left_drive.setPower(getFloat());
right_drive.setPower(getFloat());
winch.setPower(getFloat());
shoulder.setPower(getFloat());
intake.setPower(getFloat());
hand.setPosition(getFloat());
slide.setPosition(getFloat());
hook_left.setPosition(getFloat());
hook_right.setPosition(getFloat());
telemetry.addData("Indicator:", getInt());
telemetry.addData("beacon right:", (getInt() == 1 ? "red" : "blue"));
telemetry.addData("heading:", getExistingFloat(28));

}
/* Start Motor Definitions */
DeviceInterfaceModule dim;
IMU imu;int elbow_potentiometer_port = 7;

DcMotor left_drive;
DcMotor right_drive;
DcMotor shoulder;
DcMotor winch;
DcMotor intake;

Servo hand;
Servo slide;
Servo hook_left;
Servo hook_right;
/* End Motor Definitions */
native void main();

static
{
    System.loadLibrary("native_robot");
}

@Override public void runOpMode() throws InterruptedException
{
dim = hardwareMap.deviceInterfaceModule.get("dim");
I2cDevice imu_i2c_device = hardwareMap.i2cDevice.get("imu");
imu = new IMU(imu_i2c_device);
int error = imu.init(IMU.mode_ndof,
        (byte) (IMU.units_acc_m_per_s2 |
                IMU.units_angle_deg |
                IMU.units_angular_vel_deg_per_s |
                IMU.units_temp_C |
                IMU.units_pitch_convention_android));
if (error != 0) {
    for (; ; ) {
        telemetry.addData("error initializing imu", 0);
        waitOneFullHardwareCycle();
    }
}
imu.vel_x = 0.0f;
imu.vel_y = 0.0f;
imu.vel_z = 0.0f; 

left_drive  = hardwareMap.dcMotor.get("leftd");
right_drive = hardwareMap.dcMotor.get("rightd");
shoulder    = hardwareMap.dcMotor.get("shoulder");
winch       = hardwareMap.dcMotor.get("winch");
intake      = hardwareMap.dcMotor.get("intake");
right_drive.setDirection(DcMotor.Direction.REVERSE);
left_drive.setDirection(DcMotor.Direction.REVERSE);
shoulder.setDirection(DcMotor.Direction.REVERSE);
shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);
waitOneFullHardwareCycle();
intake.setDirection(DcMotor.Direction.REVERSE);
shoulder.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
winch.setMode(DcMotorController.RunMode.RESET_ENCODERS);
waitOneFullHardwareCycle();
winch.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
waitOneFullHardwareCycle();
//shoulder.setDirection(DcMotor.Direction.REVERSE);
//elbow.setDirection(DcMotor.Direction.REVERSE);

hand = hardwareMap.servo.get("hand");
slide = hardwareMap.servo.get("slide");
hook_left = hardwareMap.servo.get("hook_left");
hook_right = hardwareMap.servo.get("hook_right");
hook_left.setDirection(Servo.Direction.REVERSE);while (!FtcRobotControllerActivity.aligned || (!FtcRobotControllerActivity.red && !FtcRobotControllerActivity.blue))
{
    telemetry.addData("unchecked boxes", "fix it");
    waitForNextHardwareCycle();
}
waitForStart();
imu.rezero();

main();
}
}
