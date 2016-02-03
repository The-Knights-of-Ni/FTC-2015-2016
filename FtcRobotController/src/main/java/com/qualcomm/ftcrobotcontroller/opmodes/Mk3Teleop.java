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
import com.qualcomm.robotcore.hardware.Servo;


public class Mk3Teleop extends LinearOpMode {
public static byte[] robot_state;
int rsid_current = 0;
public Mk3Teleop()
{
    DbgLog.error("opmode constructor");
    robot_state = new byte[128];
}


public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.
{
    ByteBuffer stick = ByteBuffer.allocate(45);
    stick.put(joystick);
    return stick.getInt(40);//Offset value
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
left_drive.setPower(getFloat(28));;
right_drive.setPower(getFloat(32));;
winch.setPower(getFloat(36));;
shoulder.setPower(getFloat(40));;
intake.setPower(getFloat(44));;
hand.setPosition(getFloat(48));
slide.setPosition(getFloat(52));
hook_left.setPosition(getFloat(56));
hook_right.setPosition(getFloat(60));
telemetry.addData("shoulder theta", getFloat(64));
telemetry.addData("forearm theta", getFloat(68));

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
setInt(24, (FtcRobotControllerActivity.red ? 1 : 0));

rsid_current = 28;
}
{
rsid_current = 72;
int gamepad1_buttons = 0;
try
{
    gamepad1_buttons = updateButtons(gamepad1.toByteArray());
}
catch (RobotCoreException e)
{
    e.printStackTrace();
}
setRelative(gamepad1.left_stick_x);
setRelative( gamepad1.left_stick_y);
setRelative(gamepad1.right_stick_x);
setRelative( gamepad1.right_stick_y);
setRelative(gamepad1.left_trigger);
setRelative( gamepad1.right_trigger);
setRelative( gamepad1_buttons);
;
}
{
rsid_current = 100;
int gamepad2_buttons = 0;
try
{
    gamepad2_buttons = updateButtons(gamepad2.toByteArray());
}
catch (RobotCoreException e)
{
    e.printStackTrace();
}
setRelative(gamepad2.left_stick_x);
setRelative( gamepad2.left_stick_y);
setRelative(gamepad2.right_stick_x);
setRelative( gamepad2.right_stick_y);
setRelative(gamepad2.left_trigger);
setRelative( gamepad2.right_trigger);
setRelative( gamepad2_buttons);
;
}

}
/* Start Motor Definitions */
int elbow_potentiometer_port = 7;
DeviceInterfaceModule dim;

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
    rsid_current = 0;
dim = hardwareMap.deviceInterfaceModule.get("dim");
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
hook_left.setDirection(Servo.Direction.REVERSE);
    main();
}
}
