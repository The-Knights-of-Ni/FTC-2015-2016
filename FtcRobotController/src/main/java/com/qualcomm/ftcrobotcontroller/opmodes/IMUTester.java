package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

/**
 * Created by Dev on 12/6/2015.
 */
public class IMUTester extends LinearOpMode
{
    DeviceInterfaceModule dim;
    IMU imu;
    @Override
    public void runOpMode()
            throws InterruptedException
    {
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        imu = new IMU(0, dim);
        imu.init(IMU.mode_ndof,
                (byte)(IMU.units_acc_m_per_s2 | IMU.units_angle_rad | IMU.units_angular_vel_rad_per_s | IMU.units_temp_C | IMU.units_pitch_convention_android));
        waitForStart();
        for(;;)
        {
            short heading = imu.getEulerHeading();
            int ls = dim.getAnalogInputValue(0);
            telemetry.addData("limit switch:", ls);
            telemetry.addData("IMU heading:", heading);
            waitForNextHardwareCycle();
        }
    }
}
