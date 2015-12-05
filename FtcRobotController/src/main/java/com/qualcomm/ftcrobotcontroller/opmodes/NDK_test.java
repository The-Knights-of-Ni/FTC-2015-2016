package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.opmodes.IK_solver;
import com.qualcomm.ftcrobotcontroller.opmodes.testRobotStateElements;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class NDK_test extends LinearOpMode
{
    byte[] robot_state;
    
    DcMotor left_drive;
    DcMotor right_drive;
    
    public NDK_test() {
        robot_state = new byte[rsid_size];
    }
    
    native void main();
    
    static
    {
        System.loadLibrary("test");
    }
    
    public static final int rsid_left_drive_power = 0;
    public static final int rsid_right_drive_power = 4;
    public static final int rsid_gamepad1_left_stick_x = 8;
    public static final int rsid_gamepad1_left_stick_y = 12;
    public static final int rsid_size = 16;
    
    float getRobotStateFloat(int rsid)
    {
        return ByteBuffer.wrap(robot_state, rsid, 4).order(ByteOrder.nativeOrder()).getFloat();
    }
    
    void setRobotStateFloat(int rsid, float value)
    {
        ByteBuffer.allocateDirect(4).putFloat(value).array();
    }
    
    void applyRobotState()
    {
        setRobotStateFloat(test_rsid.gamepad1_left_stick_x, gamepad1.left_stick_x);
        setRobotStateFloat(test_rsid.gamepad1_left_stick_y, gamepad1.left_stick_y);
        
        left_drive.setPower(getRobotStateFloat(test_rsid.left_drive_power));
        right_drive.setPower(getRobotStateFloat(test_rsid.right_drive_power));
    }
    
    @Override public void runOpMode()
        throws InterruptedException
    {
        main();
        
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
