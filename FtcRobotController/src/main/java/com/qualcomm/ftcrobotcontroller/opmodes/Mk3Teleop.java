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
import com.qualcomm.robotcore.hardware.Servo;


public class Mk3Teleop extends LinearOpMode {
byte[] robot_state;
int rsid_current = 0;
public Mk3Teleop()
{
    robot_state = new byte[128];
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
setInt((FtcRobotControllerActivity.red ? 1 : 0));

rsid_current = 28;
}
{
int gamepad1_buttons = 0;
try
{
    gamepad1_buttons = updateButtons(gamepad1.toByteArray());
}
catch (RobotCoreException e)
{
    e.printStackTrace();
}
set(gamepad1.left_stick_x);
set( gamepad1.left_stick_y);
set(gamepad1.right_stick_x);
set( gamepad1.right_stick_y);
set(gamepad1.left_trigger);
set( gamepad1.right_trigger);
set( gamepad1_buttons);
;
}
{
int gamepad2_buttons = 0;
try
{
    gamepad2_buttons = updateButtons(gamepad2.toByteArray());
}
catch (RobotCoreException e)
{
    e.printStackTrace();
}
set(gamepad2.left_stick_x);
set( gamepad2.left_stick_y);
set(gamepad2.right_stick_x);
set( gamepad2.right_stick_y);
set(gamepad2.left_trigger);
set( gamepad2.right_trigger);
set( gamepad2_buttons);
;
}

}

void robotStateOut()
{
left_drive.setPower(getFloat());;
right_drive.setPower(getFloat());;
winch.setPower(getFloat());;
shoulder.setPower(getFloat());;
intake.setPower(getFloat());;
hand.setPosition(getFloat());
slide.setPosition(getFloat());
hook_left.setPosition(getFloat());
hook_right.setPosition(getFloat());
telemetry.addData("shoulder theta", getFloat());
telemetry.addData("forearm theta", getFloat());

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
