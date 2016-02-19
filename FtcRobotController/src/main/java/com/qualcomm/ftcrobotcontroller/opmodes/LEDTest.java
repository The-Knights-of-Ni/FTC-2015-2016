package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;


/**
 * Created by Dev on 2/16/2016.
 */
public class LEDTest extends LinearOpMode{
    DeviceInterfaceModule dim;
    public final byte[] datTest = {0};

    @Override
    public void runOpMode() throws InterruptedException {
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        //LED strip = new LED(hardwareMap.i2cDevice.get("led"));

        waitForStart();
        dim.setLED(0, false);
        dim.setLED(1, false);
        //strip.ledFrame();
        //strip.ledFrame();
        boolean byte0_toggled = false;
        boolean byte1_toggled = true;
        for(;;)
        {
            /*
            *if(FtcRobotControllerActivity.byte0_toggle && !byte0_toggled){
                byte0_toggled = true;
                strip.write(0xFFFF, datTest);
                telemetry.addData("Byte 0","happened");
            }
            else{
                byte0_toggled = false;
            }
            if(FtcRobotControllerActivity.byte1_toggle && !byte1_toggled){
                byte1_toggled = true;
                strip.write(0x0, datTest);
                telemetry.addData("Byte 1", "happened");
            }
            else
            {
                byte1_toggled = false;
            }*/
            if(FtcRobotControllerActivity.red)
            {
                dim.setLED(0, false);
                dim.setLED(1, true);
            }
            else if(FtcRobotControllerActivity.blue)
            {
                dim.setLED(0, true);
                dim.setLED(1, false);
            }
            else
            {
                dim.setLED(0, false);
                dim.setLED(1, false);
            }
            telemetry.addData("Complete","d");
            telemetry.addData("Switch", dim.getDigitalInputStateByte());
            waitOneFullHardwareCycle();
        }
    }
}
