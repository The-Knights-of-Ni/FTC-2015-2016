package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftcrobotcontroller.opmodes.Mk3TeleopRobotStateElements;

import java.nio.ByteBuffer;

/**
 * Created by Dev on 1/4/2016.
 */
public class Mk3Teleop extends LinearOpMode {
    /* Start NDK Stuff*/
    byte[] robot_state;

    //TODO: Write state element file
    public Mk3Teleop()
    {
        robot_state = new byte[Mk3TeleopRobotStateElements.robot_state_size];
        Mk3TeleopRobotStateElements.robot_state = robot_state;
    }

    native void main();

    static
    {
        System.loadLibrary("Mk3Teleop");
    }

    void applyRobotState()
    {
        //TODO: Add stuff we want to bring into C here. IMU, Gamepads, Buttons
        /*DATA OUT (To native)*/
        //Gamepad 1
        Mk3TeleopRobotStateElements.set_gamepad1_joystick1_x(gamepad1.left_stick_x);
        Mk3TeleopRobotStateElements.set_gamepad1_joystick1_y(gamepad1.left_stick_y);
        Mk3TeleopRobotStateElements.set_gamepad1_joystick2_x(gamepad1.right_stick_x);
        Mk3TeleopRobotStateElements.set_gamepad1_joystick2_y(gamepad1.right_stick_y);
        Mk3TeleopRobotStateElements.set_gamepad1_trigger1(gamepad1.left_trigger);
        Mk3TeleopRobotStateElements.set_gamepad1_trigger2(gamepad1.right_trigger);
        try {
            Mk3TeleopRobotStateElements.set_gamepad1_buttons(updateButtons(gamepad1.toByteArray()));
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
        //Gamepad 2
        Mk3TeleopRobotStateElements.set_gamepad2_joystick1_x(gamepad2.left_stick_x);
        Mk3TeleopRobotStateElements.set_gamepad2_joystick1_y(gamepad2.left_stick_y);
        Mk3TeleopRobotStateElements.set_gamepad2_joystick2_x(gamepad2.right_stick_x);
        Mk3TeleopRobotStateElements.set_gamepad2_joystick2_y(gamepad2.right_stick_y);
        Mk3TeleopRobotStateElements.set_gamepad2_trigger1(gamepad2.left_trigger);
        Mk3TeleopRobotStateElements.set_gamepad2_trigger2(gamepad2.right_trigger);
        try {
            Mk3TeleopRobotStateElements.set_gamepad1_buttons(updateButtons(gamepad2.toByteArray()));
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }

        /*DATA IN (From native)*/
        left_drive.setPower(Mk3TeleopRobotStateElements.get_left_drive());
        right_drive.setPower(Mk3TeleopRobotStateElements.get_right_drive());
        elbow.setPower(Mk3TeleopRobotStateElements.get_elbow());
        shoulder.setPower(Mk3TeleopRobotStateElements.get_shoulder());
        intake.setPower(Mk3TeleopRobotStateElements.get_intake());
        hand_servo.setPosition(Mk3TeleopRobotStateElements.get_hand());
        slide_servo.setPosition(Mk3TeleopRobotStateElements.get_slide());
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
    Servo slide_servo;
    /* End Motor Definitions */

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
        slide_servo = hardwareMap.servo.get("servo_2");

        waitForStart();


        main();

    }
}
