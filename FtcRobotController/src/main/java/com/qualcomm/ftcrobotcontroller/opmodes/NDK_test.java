package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.opmodes.testRobotStateElements;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftccommon.DbgLog;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class NDK_test extends LinearOpMode
{
    byte[] robot_state;
    
    /* DcMotor left_drive; */
    /* DcMotor right_drive; */
    
    public NDK_test() {
        robot_state = new byte[testRobotStateElements.robot_state_size];
        testRobotStateElements.robot_state = robot_state;
    }
    
    native void main();
    
    static
    {
        System.loadLibrary("test");
    }
    
    
    void applyRobotState()
    {
        testRobotStateElements.set_gamepad1_joystick_1_x(gamepad1.left_stick_x);
        testRobotStateElements.set_gamepad1_joystick_1_y(gamepad1.left_stick_y);
        
        telemetry.addData("left_drive_power", String.format("%.2f", testRobotStateElements.get_left_drive_power()));
        telemetry.addData("right_drive_power", String.format("%.2f", testRobotStateElements.get_right_drive_power()));
    }
    
    @Override public void runOpMode()
        throws InterruptedException
    {
        DbgLog.error("NDK test init");
        waitForStart();
        main();
        for(;;)
        {
            telemetry.addData("waiting to die", "hi");
            waitOneFullHardwareCycle();
        }
        
        //Java equivalent
        
        /* left_drive  = hardwareMap.dcMotor.get("left_d"); */
        /* right_drive = hardwareMap.dcMotor.get("right_d"); */
        /* right_drive.setDirection(DcMotor.Direction.REVERSE); */
        
        /* waitForStart(); */
        /* for(;;) */
        /* { */
        /*     //drive */
        /*     float[] drive_stick = new float[]{-gamepad1.left_stick_x, -gamepad1.left_stick_y}; */
        /*     deadZone(drive_stick); */
        
        /*     float left_power = drive_stick[1]-drive_stick[0]; */
        /*     float right_power = drive_stick[1]+drive_stick[0]; */
        /*     right_power = Range.clip(right_power, -1, 1); */
        /*     left_power = Range.clip(left_power, -1, 1); */
        /*     right_drive.setPower(right_power); */
        /*     left_drive.setPower(left_power); */
        
        
        /*     waitOneFullHardwareCycle(); */
        /* } */
    }
}
