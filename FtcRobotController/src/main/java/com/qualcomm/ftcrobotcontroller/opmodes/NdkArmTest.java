package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.opmodes.arm_testRobotStateElements;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftccommon.DbgLog;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class NdkArmTest extends LinearOpMode
{
    byte[] robot_state;
    
    DeviceInterfaceModule dim;
    int elbow_potentiometer_port;
    
    DcMotor shoulder;
    DcMotor winch;
    DcMotor intake;
    
    public NdkArmTest()
    {
        robot_state = new byte[arm_testRobotStateElements.robot_state_size];
        arm_testRobotStateElements.robot_state = robot_state;
    }
    
    native void main();
    
    static
    {
        System.loadLibrary("arm_test");
    }

    public static final float encoder_ticks_per_radian = 1440.0f/(2.0f*(float)Math.PI); //TODO: might want to make this a global const
    public static final float potentiometer_range = 333.33333333333333333333333333333333333f;
    float elbow_potentiometer_angle;
    double last_time = time;
    
    float lerp(float a, float b, float t)
    {
        if(t > 1.0f) return b;
        if(t < 0.0f) return a;
        return a+(b-a)*t;
    }
    
    public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.
    {
        ByteBuffer stick = ByteBuffer.allocate(45);
        stick.put(joystick);
        return stick.getInt(40);//Offset value
    }
    
    void applyRobotState()
    {        
        double this_time = time;
        double dt = this_time-last_time;
        last_time = this_time;
        arm_testRobotStateElements.set_time(this_time);
        
        //Gamepad 1
        arm_testRobotStateElements.set_gamepad1_joystick1_x(gamepad1.left_stick_x);
        arm_testRobotStateElements.set_gamepad1_joystick1_y(gamepad1.left_stick_y);
        arm_testRobotStateElements.set_gamepad1_joystick2_x(gamepad1.right_stick_x);
        arm_testRobotStateElements.set_gamepad1_joystick2_y(gamepad1.right_stick_y);
        arm_testRobotStateElements.set_gamepad1_left_trigger(gamepad1.left_trigger);
        arm_testRobotStateElements.set_gamepad1_right_trigger(gamepad1.right_trigger);
        try {
            arm_testRobotStateElements.set_gamepad1_buttons(updateButtons(gamepad1.toByteArray()));
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
        //Gamepad 2
        arm_testRobotStateElements.set_gamepad2_joystick1_x(gamepad2.left_stick_x);
        arm_testRobotStateElements.set_gamepad2_joystick1_y(gamepad2.left_stick_y);
        arm_testRobotStateElements.set_gamepad2_joystick2_x(gamepad2.right_stick_x);
        arm_testRobotStateElements.set_gamepad2_joystick2_y(gamepad2.right_stick_y);
        arm_testRobotStateElements.set_gamepad2_left_trigger(gamepad2.left_trigger);
        arm_testRobotStateElements.set_gamepad2_right_trigger(gamepad2.right_trigger);
        try {
            arm_testRobotStateElements.set_gamepad1_buttons(updateButtons(gamepad2.toByteArray()));
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
        
        //TODO; probably should do scaling and filtering in native
        elbow_potentiometer_angle = lerp(
            (((180.0f-potentiometer_range*0.5f+potentiometer_range*(((float)dim.getAnalogInputValue(elbow_potentiometer_port))/(1023.0f)))+12.0f))*(float) Math.PI/180.0f,
            elbow_potentiometer_angle,
            (float)Math.exp(-20.0*dt));
        
        arm_testRobotStateElements.set_shoulder_encoder(shoulder.getCurrentPosition()/encoder_ticks_per_radian);
        arm_testRobotStateElements.set_elbow_potentiometer(elbow_potentiometer_angle);
        arm_testRobotStateElements.set_winch_encoder(winch.getCurrentPosition()/encoder_ticks_per_radian);
        
        shoulder.setPower(arm_testRobotStateElements.get_arm_shoulder_power());
        winch.setPower(arm_testRobotStateElements.get_arm_winch_power());
        intake.setPower(arm_testRobotStateElements.get_arm_intake_power());

        telemetry.addData("intake power", String.format("%.2f", arm_testRobotStateElements.get_arm_intake_power()));

    }
    
    @Override public void runOpMode()
        throws InterruptedException
    {
        elbow_potentiometer_port = 7;
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        shoulder = hardwareMap.dcMotor.get("shoulder");
        winch = hardwareMap.dcMotor.get("winch");
        intake = hardwareMap.dcMotor.get("intake");
        
        shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        winch.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        main();
    }
}
