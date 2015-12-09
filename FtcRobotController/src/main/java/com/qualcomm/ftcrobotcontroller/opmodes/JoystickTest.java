package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.ftcrobotcontroller.opmodes.Button;
import com.qualcomm.robotcore.exception.RobotCoreException;

import java.nio.ByteBuffer;

/**
 * Created by Dev on 12/7/2015.
 */
public class JoystickTest extends LinearOpMode
{
    public void runOpMode()
    {
        Button joystick1 = new Button();//Button should be for each controller
        Button joystick2 = new Button();

        boolean toggle = false;
        boolean press;

        for(;;)
        {
            boolean press2 = joystick1.press(Button.Buttons.LEFT_STICK_BUTTON);
            toggle = joystick1.toggle(Button.Buttons.X);
            //press = joystick1.release(Button.Buttons.A);
            telemetry.addData("X Toggle", toggle);
            //telemetry.addData("PrevByte",);
            telemetry.addData("Press 2", press2);
            //System.out.println(joystick1.previousByte);
            try
            {
                joystick1.updateButtons(gamepad1.toByteArray());
                joystick2.updateButtons(gamepad2.toByteArray());
            } catch (RobotCoreException e)
            {
                e.printStackTrace();
            }
        }
    }

}

