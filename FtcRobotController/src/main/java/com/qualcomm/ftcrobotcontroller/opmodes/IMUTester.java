package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.ftccommon.DbgLog;

public class IMUTester extends LinearOpMode
{
    DeviceInterfaceModule dim;
    IMU imu;
    
    float lerp(float a, float b, float t)
    {
        if(t > 1.0f) return b;
        if(t < 0.0f) return a;
        return a+(b-a)*t;
    }
    
    @Override
    public void runOpMode()
            throws InterruptedException
    {
        resetStartTime();
        
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        I2cDevice imu_i2c_device = hardwareMap.i2cDevice.get("imu");
        
        imu = new IMU(imu_i2c_device);
        
        int error = imu.init(IMU.mode_ndof,
                             (byte)(IMU.units_acc_m_per_s2|
                                    IMU.units_angle_deg |
                                    IMU.units_angular_vel_deg_per_s |
                                    IMU.units_temp_C |
                                    IMU.units_pitch_convention_android));

        if(error != 0)
        {
            for(;;)
            {
                telemetry.addData("error initializing imu", 0);
                waitOneFullHardwareCycle();
            }
        }
        
        /* short heading; */
        /* short[] acceleration = new short[3]; */
        /* float[] high_passed_acceleration = new float[3]; */
        /* float[] velocity = new float[3]; */
        /* while(!imu.checkForUpdate()){imu.delay(100);} */
        /* heading = imu.getEulerHeading(); */
        /* acceleration[0] = imu.getAccelerationX(); */
        /* acceleration[1] = imu.getAccelerationY(); */
        /* acceleration[2] = imu.getAccelerationZ(); */
        /* for(int i = 0; i < 3; i++) high_passed_acceleration[i] = acceleration[i]; */
        /* for(int i = 0; i < 3; i++) velocity[i] = 0; */
        
        telemetry.addData("waiting for start, init time in seconds", time);
        
        double old_time = 0.0;
        resetStartTime();
        
        /* double last_update_time = 0.0; */
        /* double update_cycle_time = 0.0; */
        
        DbgLog.error("waiting for start");
        waitForStart();
        for(;;)
        {
            double new_time = time;
            double dt = new_time-old_time;
            old_time = new_time;
            /* if(imu.checkForUpdate()) */
            /* { */
            /*     double update_dt = new_time-last_update_time; */
            /*     last_update_time = new_time; */
            /*     update_cycle_time = update_dt; */
                
            /*     heading = imu.getEulerHeading(); */
            /*     acceleration[0] = imu.getAccelerationX(); */
            /*     acceleration[1] = imu.getAccelerationY(); */
            /*     acceleration[2] = imu.getAccelerationZ(); */
            /*     for(int i = 0; i < 3; i++) velocity[i] += high_passed_acceleration[i]*update_dt; */
            /*     for(int i = 0; i < 3; i++) high_passed_acceleration[i] = lerp(high_passed_acceleration[i], */
            /*                                                                   (float)acceleration[i], */
            /*                                                                   (float)Math.exp(-100.0*update_dt)); */
            /* } */
            int ls = dim.getAnalogInputValue(0);
            telemetry.addData("limit switch", ls);
            /* try */
            /* { */
            /*     imu.read_lock.lock(); */
                telemetry.addData("IMU heading", imu.eul_x);
                telemetry.addData("acceleration x", imu.lia_x);
                telemetry.addData("acceleration y", imu.lia_y);
                telemetry.addData("acceleration z", imu.lia_z);
                telemetry.addData("IMU cycle time (ns)", ((float)imu.dt)/1000000000.0);
            /* } */
            /* finally */
            /* { */
            /*     imu.read_lock.unlock(); */
            /* } */
            /* telemetry.addData("high_passed_acceleration x", high_passed_acceleration[0]); */
            /* telemetry.addData("high_passed_acceleration y", high_passed_acceleration[1]); */
            /* telemetry.addData("high_passed_acceleration z", high_passed_acceleration[2]); */
            /* telemetry.addData("velocity x", velocity[0]); */
            /* telemetry.addData("velocity y", velocity[1]); */
            /* telemetry.addData("velocity z", velocity[2]); */
            telemetry.addData("cycle time", dt);
            waitForNextHardwareCycle();
        }
    }
}
