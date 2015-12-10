package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.ftccommon.DbgLog;

public class IMUTester extends LinearOpMode
{
    DeviceInterfaceModule dim;
    IMU imu;
    @Override
    public void runOpMode()
            throws InterruptedException
    {
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        I2cDevice imu_i2c_device = hardwareMap.i2cDevice.get("imu");
        
        imu = new IMU(0, imu_i2c_device);
        
        byte[] registers_to_read = new byte[]{
            IMU.EUL_DATA_X, IMU.EUL_DATA_X+2,
        };
        int error = imu.init(IMU.mode_ndof,
                             (byte)(IMU.units_acc_m_per_s2|
                                    IMU.units_angle_deg |
                                    IMU.units_angular_vel_rad_per_s |
                                    IMU.units_temp_C |
                                    IMU.units_pitch_convention_android),
                             registers_to_read);
        
        if(error == 0)
        {
            DbgLog.error("waiting for start");
            waitForStart();
            for(;;)
            {
                short heading = imu.getEulerHeading();
                int ls = dim.getAnalogInputValue(0);
                telemetry.addData("limit switch:", ls);
                telemetry.addData("IMU heading:", heading);
                waitOneFullHardwareCycle();
            }
        }
        else
        {
            for(;;)
            {
                telemetry.addData("error initializing imu", 0);
            }
        }
    }
}
