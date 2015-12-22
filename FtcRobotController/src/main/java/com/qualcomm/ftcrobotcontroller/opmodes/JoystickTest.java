package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.ftcrobotcontroller.opmodes.Button;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.ftcrobotcontroller.opmodes.Drive;
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

        float heading = 90;

        for(;;)
        {
            /*
            TODO: Fix heading change for 180 to -180 so that it uses an optimal path.
            gamepad1.setJoystickDeadzone( (float) 0.1);
            boolean reverseDrive = joystick1.toggle(Button.Buttons.LEFT_STICK_BUTTON);
            double magnitude = Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y);
            double theta = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x)*180/Math.PI;//Using degrees for now.
            float throttle = gamepad1.left_trigger * (reverseDrive ? -1 : 1);
            if(magnitude != 0)
            {
                if (theta > heading)
                    heading++;
                if (heading > theta)
                    heading--;
            }




            telemetry.addData("Mag", magnitude);
            telemetry.addData("Theta", theta);
            telemetry.addData("Throttle", throttle);
            telemetry.addData("Heading", heading);
*/

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

