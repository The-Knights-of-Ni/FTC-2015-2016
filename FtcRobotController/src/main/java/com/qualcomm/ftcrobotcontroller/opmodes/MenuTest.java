package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Dev on 1/18/2016.
 */
public class MenuTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        while(true)

        {
            telemetry.addData("aligned: ", FtcRobotControllerActivity.aligned);
            telemetry.addData("red: ", FtcRobotControllerActivity.red);
            telemetry.addData("blue: ", FtcRobotControllerActivity.blue);
            waitOneFullHardwareCycle();
        }
    }
}
