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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import android.hardware.Camera;


public class Mk4Auto extends LinearOpMode {
public static byte[] robot_state;
int rsid_current = 0;
public Mk4Auto()
{
    DbgLog.error("opmode constructor");
    robot_state = new byte[140];

camera = FtcRobotControllerActivity.camera_preview.camera;
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
left_drive.setPower(getFloat(68));
right_drive.setPower(getFloat(72));
winch.setPower(getFloat(76));
shoulder.setPower(getFloat(80));
intake.setPower(getFloat(84));
hand.setPosition(getFloat(88));
wrist.setPosition(getFloat(92));
hook_left.setPosition(getFloat(96));
hook_right.setPosition(getFloat(100));
intake_tilt.setPosition(getFloat(104));
telemetry.addData("Indicator:", getInt(108));
telemetry.addData("left_drive_encoder:", getInt(12));
telemetry.addData("right_drive_encoder:", getInt(8));
telemetry.addData("beacon right:", (getInt(112) == 1 ? "red" : "blue"));
telemetry.addData("heading:", getShort(44));
telemetry.addData("target time:", getFloat(116));
telemetry.addData("acceleration time:", getFloat(120));
telemetry.addData("slider 0", getInt(124));
telemetry.addData("slider 1", getInt(128));
telemetry.addData("slider 2", getInt(132));
telemetry.addData("slider 3", getInt(136));

}

void robotStateIn()
{
{
setDouble(0, time);

rsid_current = 8;
}
{
setInt(8, right_drive.getCurrentPosition());

rsid_current = 12;
}
{
setInt(12, left_drive.getCurrentPosition());

rsid_current = 16;
}
{
setInt(16, winch.getCurrentPosition());

rsid_current = 20;
}
{
setInt(20, shoulder.getCurrentPosition());

rsid_current = 24;
}
{
setInt(24, dim.getAnalogInputValue(elbow_potentiometer_port));

rsid_current = 28;
}
{
setInt(28, dim.getAnalogInputValue(shoulder_potentiometer_port));

rsid_current = 32;
}
{
setFloat(32, (float)left_drive_voltage.getVoltage());

rsid_current = 36;
}
{
setFloat(36, (float)right_drive_voltage.getVoltage());

rsid_current = 40;
}
{
setInt(40, dim.getDigitalInputStateByte());

rsid_current = 44;
}
{
rsid_current = 44;
if(imu.checkForUpdate()) {
    setRelative(imu.eul_x);
setRelative( imu.eul_y);
setRelative( imu.eul_z);
setRelative( imu.vel_x);
setRelative( imu.vel_y);
setRelative( imu.vel_z);
;
}

}
{
setInt(64, (FtcRobotControllerActivity.red ? 1 : 0));

rsid_current = 68;
}
{
setInt(124, FtcRobotControllerActivity.slider_0);

rsid_current = 128;
}
{
setInt(128, FtcRobotControllerActivity.slider_1);

rsid_current = 132;
}
{
setInt(132, FtcRobotControllerActivity.slider_2);

rsid_current = 136;
}
{
setInt(136, FtcRobotControllerActivity.slider_3);

rsid_current = 140;
}

}
/* Start Motor Definitions */
VoltageSensor left_drive_voltage;
VoltageSensor right_drive_voltage;
DeviceInterfaceModule dim;
IMU imu;int elbow_potentiometer_port = 7;
int shoulder_potentiometer_port = 1;

DcMotor left_drive;
DcMotor right_drive;
DcMotor shoulder;
DcMotor winch;
DcMotor intake;

Servo hand;
Servo wrist;
Servo hook_left;
Servo hook_right;
Servo intake_tilt;
/* End Motor Definitions */

native void main();

static
{
    System.loadLibrary("native_robot");
}

@Override public void runOpMode() throws InterruptedException
{
left_drive_voltage = hardwareMap.voltageSensor.get("Left Drive + Shoulder");
right_drive_voltage = hardwareMap.voltageSensor.get("Intake + Right Drive");
dim = hardwareMap.deviceInterfaceModule.get("dim");
I2cDevice imu_i2c_device = hardwareMap.i2cDevice.get("imu");
imu = new IMU(imu_i2c_device, this);
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
//right_drive.setDirection(DcMotor.Direction.REVERSE);
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
shoulder.setDirection(DcMotor.Direction.REVERSE);
//winch.setDirection(DcMotor.Direction.REVERSE);

hand = hardwareMap.servo.get("hand");
hand.setDirection(Servo.Direction.REVERSE);
wrist = hardwareMap.servo.get("wrist");
hook_left = hardwareMap.servo.get("hook_left");
hook_right = hardwareMap.servo.get("hook_right");
hook_left.setDirection(Servo.Direction.REVERSE);
intake_tilt = hardwareMap.servo.get("intake_tilt");
//intake_tilt.setDirection(Servo.Direction.REVERSE);
dim.setLED(0, false);
dim.setLED(1, false);
while (!FtcRobotControllerActivity.aligned || (!FtcRobotControllerActivity.red && !FtcRobotControllerActivity.blue))
{
    telemetry.addData("unchecked boxes", "fix it");
    waitForNextHardwareCycle();
}
if(FtcRobotControllerActivity.red)
{
    dim.setLED(0, false);
    dim.setLED(1, true);
}
else if(FtcRobotControllerActivity.blue)
{
    dim.setLED(0, true);
    dim.setLED(1, false);
}
else
{
    dim.setLED(0, false);
    dim.setLED(1, false);
}telemetry.addData("ready", "");
waitForStart();
imu.rezero();

    main();
}
}
