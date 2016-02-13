package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

/**
 * Created by Dev on 2/12/2016.
 */
public class fsrTest extends LinearOpMode {
    DeviceInterfaceModule dim;

    @Override
    public void runOpMode() throws InterruptedException {
        int fsrPort = 0;
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        waitOneFullHardwareCycle();
        while(true)
        {
            telemetry.addData("FSR Reads:", dim.getAnalogInputValue(fsrPort));
            waitOneFullHardwareCycle();
        }
    }
}
