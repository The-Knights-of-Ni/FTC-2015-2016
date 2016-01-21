package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftcrobotcontroller.opmodes.Mk3AutoRobotStateElements;

import java.nio.ByteBuffer;


public class Mk3Auto extends LinearOpMode {
    /* Start NDK Stuff*/
    byte[] robot_state;
    int elbow_potentiometer_port = 7;
    
    public Mk3Auto() {
        robot_state = new byte[Mk3AutoRobotStateElements.robot_state_size];
        Mk3AutoRobotStateElements.robot_state = robot_state;
    }
    
    native void main();
    
    static {
        System.loadLibrary("Mk3Auto");
    }
    
    void applyRobotState() {
        /*DATA OUT (To native)*/
        //Sensors
       Mk3AutoRobotStateElements.set_right_drive_encoder(right_drive.getCurrentPosition());
       Mk3AutoRobotStateElements.set_left_drive_encoder(left_drive.getCurrentPosition());
       Mk3AutoRobotStateElements.set_winch_encoder(winch.getCurrentPosition());
       Mk3AutoRobotStateElements.set_shoulder_encoder(shoulder.getCurrentPosition());
       Mk3AutoRobotStateElements.set_elbow_potentiometer(dim.getAnalogInputValue(elbow_potentiometer_port));
       Mk3AutoRobotStateElements.set_imu_heading(6);//TODO: get these from the imu
       Mk3AutoRobotStateElements.set_imu_tilt(6);
       Mk3AutoRobotStateElements.set_imu_roll(6);
       Mk3AutoRobotStateElements.set_imu_velocity_x(6);
       Mk3AutoRobotStateElements.set_imu_velocity_y(6);
       
       /*DATA IN (From native)*/
       left_drive.setPower(Mk3AutoRobotStateElements.get_left_drive());
       right_drive.setPower(Mk3AutoRobotStateElements.get_right_drive());
       winch.setPower(Mk3AutoRobotStateElements.get_winch());
       shoulder.setPower(Mk3AutoRobotStateElements.get_shoulder());
       intake.setPower(Mk3AutoRobotStateElements.get_intake());
       hand_servo.setPosition(Mk3AutoRobotStateElements.get_hand());
       slide_servo.setPosition(Mk3AutoRobotStateElements.get_slide());
        telemetry.addData("Indicator:", Mk3AutoRobotStateElements.get_indicator());
    }

    /* End NDK Stuff*/


    /* Start Motor Definitions */
    DeviceInterfaceModule dim;

    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor shoulder;
    DcMotor winch;
    DcMotor intake;
    
    Servo hand_servo;
    Servo slide_servo;
    /* End Motor Definitions*/

    public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.
    {
        ByteBuffer stick = ByteBuffer.allocate(45);
        stick.put(joystick);
        return stick.getInt(40);//Offset value
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int elbow_potentiometer_port = 7;
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        left_drive = hardwareMap.dcMotor.get("leftd");
        right_drive = hardwareMap.dcMotor.get("rightd");
        shoulder = hardwareMap.dcMotor.get("shoulder");
        winch = hardwareMap.dcMotor.get("winch");
        intake = hardwareMap.dcMotor.get("intake");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        shoulder.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        winch.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        winch.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        
        hand_servo = hardwareMap.servo.get("hand");
        slide_servo = hardwareMap.servo.get("slide");
        
        waitForStart();
        //TODO: Add menu with checkboxes and blue/red
        
        main();
    }
}
