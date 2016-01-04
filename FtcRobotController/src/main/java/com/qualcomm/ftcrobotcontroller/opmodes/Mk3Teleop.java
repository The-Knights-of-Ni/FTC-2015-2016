package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Dev on 1/4/2016.
 */
public class Mk3Teleop extends LinearOpMode {
    /* Start NDK Stuff*/
    byte[] robot_state;

    //TODO: Write state element file
    public Mk3Teleop()
    {
        robot_state = new byte[Mk3RobotStateElements.robot_state_size];
        Mk3RobotStateElements.robot_state = robot_state;
    }

    native void main();

    static
    {
        System.loadLibrary("Mk3Teleop");
    }

    void applyRobotState()
    {
        //TODO: Add stuff we want to bring into C here. IMU, Gamepads, Buttons
    }
    /* End NDK Stuff*/


    /* Start Motor Definitions */
    DeviceInterfaceModule dim;

    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor shoulder;
    DcMotor elbow;
    DcMotor intake;

    Servo hand_servo;
    /* End Motor Definitions */

    @Override
    public void runOpMode() throws InterruptedException {
        int elbow_potentiometer_port = 7;
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        left_drive  = hardwareMap.dcMotor.get("leftd");
        right_drive = hardwareMap.dcMotor.get("rightd");
        shoulder    = hardwareMap.dcMotor.get("shoulder");
        elbow       = hardwareMap.dcMotor.get("elbow");
        intake      = hardwareMap.dcMotor.get("intake");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        shoulder.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        elbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        hand_servo = hardwareMap.servo.get("servo_1");

        waitForStart();

        Button joystick1 = new Button();//These might go in C instead.
        Button joystick2 = new Button();
        main();

    }
}
