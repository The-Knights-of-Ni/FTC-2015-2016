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
        resetStartTime();
        
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        I2cDevice imu_i2c_device = hardwareMap.i2cDevice.get("imu");
        
        imu = new IMU(imu_i2c_device);
        
        byte[] registers_to_read = new byte[]{
            IMU.ACC_DATA_X, IMU.ACC_DATA_X+1,
            IMU.ACC_DATA_Y, IMU.ACC_DATA_Y+1,
            IMU.ACC_DATA_Z, IMU.ACC_DATA_Z+1,
            IMU.EUL_DATA_X, IMU.EUL_DATA_X+1,
        };
        int error = imu.init(IMU.mode_ndof,
                             (byte)(IMU.units_acc_m_per_s2|
                                    IMU.units_angle_deg |
                                    IMU.units_angular_vel_deg_per_s |
                                    IMU.units_temp_C |
                                    IMU.units_pitch_convention_android),
                             registers_to_read);

        if(error != 0)
        {
            for(;;)
            {
                telemetry.addData("error initializing imu", 0);
            }
        }
        
        short heading;
        short[] acceleration = new short[3];
        while(!imu.checkForUpdate()){imu.delay(100);}
        heading = imu.getEulerHeading();
        acceleration[0] = imu.getAccelerationX();
        acceleration[1] = imu.getAccelerationY();
        acceleration[2] = imu.getAccelerationZ();
        
        telemetry.addData("waiting for start, init time in seconds:", time);
        
        double old_time = 0.0;
        resetStartTime();
        
        double last_update_time = 0.0;
        double update_cycle_time = 0.0;
        
        DbgLog.error("waiting for start");
        waitForStart();
        for(;;)
        {
            double new_time = time;
            if(imu.checkForUpdate())
            {
                heading = imu.getEulerHeading();
                acceleration[0] = imu.getAccelerationX();
                acceleration[1] = imu.getAccelerationY();
                acceleration[2] = imu.getAccelerationZ();
                update_cycle_time = new_time-last_update_time;
                last_update_time = new_time;
            }
            int ls = dim.getAnalogInputValue(0);
            telemetry.addData("limit switch:", ls);
            telemetry.addData("IMU heading:", heading);
            telemetry.addData("acceleration x:", acceleration[0]);
            telemetry.addData("acceleration y:", acceleration[1]);
            telemetry.addData("acceleration z:", acceleration[2]);
            telemetry.addData("cycle time:", new_time - old_time);
            telemetry.addData("IMU cycle time:", update_cycle_time);
            old_time = new_time;
            waitOneFullHardwareCycle();
        }
    }
}
