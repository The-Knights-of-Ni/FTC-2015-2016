package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
public class PrototypeDrive extends LinearOpMode
{
    DcMotor left_front_drive;
    DcMotor right_front_drive;
    DcMotor left_back_drive;
    DcMotor right_back_drive;

    public PrototypeDrive() {}

    @Override public void runOpMode()
        throws InterruptedException
    {
        left_front_drive = hardwareMap.dcMotor.get("motor_4");
        left_back_drive = hardwareMap.dcMotor.get("motor_3");
        right_front_drive = hardwareMap.dcMotor.get("motor_2");
        right_back_drive = hardwareMap.dcMotor.get("motor_1");
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();

        for (;;)
        {
            float stick_x = gamepad1.left_stick_x;
            float stick_y = -gamepad1.left_stick_y;


            float norm = (float) Math.sqrt(((stick_x)*(stick_x))+((stick_y)*(stick_y)));
            if(norm < 0.15f)
            {
                stick_x = 0;
                stick_y = 0;
            }
            else
            {
                float modifier = 1.0f;
                float speed = (norm-0.15f)/(1-0.15f)/norm*modifier;
                stick_x *= speed;
                stick_y *= speed;
            }

            float left_power = stick_y-stick_x;
            float right_power = stick_y+stick_x;

            right_power = (((right_power) < -1.0f) ? -1.0f : (((right_power) > 1.0f) ? 1.0f : (right_power)));
            left_power = (((left_power) < -1.0f) ? -1.0f : (((left_power) > 1.0f) ? 1.0f : (left_power)));

            right_front_drive.setPower(right_power);
            right_back_drive.setPower(right_power);
            left_front_drive.setPower(left_power);
            left_back_drive.setPower(left_power);

            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("left power", String.format("%.2f", left_power));
            telemetry.addData("right power", String.format("%.2f", right_power));
            telemetry.addData("raw stick x", String.format("%.2f", gamepad1.left_stick_x));
            telemetry.addData("raw stick y", String.format("%.2f", gamepad1.left_stick_y));
            telemetry.addData("stick x", String.format("%.2f", stick_x));
            telemetry.addData("stick y", String.format("%.2f", stick_y));

            waitOneFullHardwareCycle();
        }
    }
}
