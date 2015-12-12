package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.ftccommon.DbgLog;

import android.content.Context;
import java.io.File;
import java.io.FileOutputStream;
import android.os.Environment;

public class IMUTester extends LinearOpMode
{
    DeviceInterfaceModule dim;
    IMU imu;

    public boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
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

        int calibration_points = 2000;
        long calib_acc0_x = 0;
        long calib_acc0_y = 0;
        long calib_acc0_z = 0;
        
        for(int i = 0; i < calibration_points; i++)
        {
            if(imu.checkForUpdate())
            {
                calib_acc0_x += imu.lia_x;
                calib_acc0_y += imu.lia_y;
                calib_acc0_z += imu.lia_z;
                telemetry.addData("calibrating", i);
            }
            waitForNextHardwareCycle();
        }
        imu.acc0_x = (float) (((double) calib_acc0_x)/((double) calibration_points));
        imu.acc0_y = (float) (((double) calib_acc0_y)/((double) calibration_points));
        imu.acc0_z = (float) (((double) calib_acc0_z)/((double) calibration_points));
        imu.vel_x = 0.0f;
        imu.vel_y = 0.0f;
        imu.vel_z = 0.0f;
        
        telemetry.addData("waiting for start, init time in seconds", time);
        
        double old_time = 0.0;
        resetStartTime();
        
        double last_update_time = 0.0;
        double update_cycle_time = 0.0;

        String log = "";
        String log_file_info = "";
        try
        {
            if(!isExternalStorageWritable())
            {
                for(;;)
                {
                    telemetry.addData("error", "external storage is not currently writable");
                    waitOneFullHardwareCycle();
                }
            }
            String log_filename = "imu_test_log.txt";
            File logfile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), log_filename);
            FileOutputStream log_stream = new FileOutputStream(logfile);
            
            DbgLog.error("waiting for start");
            waitForStart();
            imu.vel_x = 0.0f;
            imu.vel_y = 0.0f;
            imu.vel_z = 0.0f;
            imu.dt = 0.0f;
            imu.old_time = System.nanoTime();
            imu.n_reads = 0;
            
            for(;imu.n_reads < 10000;)
            {
                double new_time = time;
                double dt = new_time-old_time;
                old_time = new_time;
                if(imu.checkForUpdate())
                {
                    update_cycle_time = new_time-last_update_time;
                    last_update_time = new_time;
                }
                
                int ls = dim.getAnalogInputValue(0);
                telemetry.addData("number of reads", imu.n_reads);
                telemetry.addData("limit switch", ls);
                telemetry.addData("IMU heading", ((float)imu.eul_x)/16.0f);
                telemetry.addData("acceleration x", imu.lia_x/100.0f);
                telemetry.addData("acceleration y", imu.lia_y/100.0f);
                telemetry.addData("acceleration z", imu.lia_z/100.0f);
                telemetry.addData("IMU cycle time 1", imu.dt);
                telemetry.addData("IMU cycle time 2", update_cycle_time);
                telemetry.addData("high_passed_acceleration x", imu.acc_x/100.0f);
                telemetry.addData("high_passed_acceleration y", imu.acc_y/100.0f);
                telemetry.addData("high_passed_acceleration z", imu.acc_z/100.0f);
                telemetry.addData("accelerometer calibration x", imu.acc0_x/100.0f);
                telemetry.addData("accelerometer calibration y", imu.acc0_y/100.0f);
                telemetry.addData("accelerometer calibration z", imu.acc0_z/100.0f);
                telemetry.addData("velocity x", imu.vel_x/100.0f);
                telemetry.addData("velocity y", imu.vel_y/100.0f);
                telemetry.addData("velocity z", imu.vel_z/100.0f);
                telemetry.addData("cycle time", dt);
                telemetry.addData("file", logfile.toString());

                log = log.concat(String.format("raw: %d, %d, %d\nhpa: %f, %f, %f\nvel: %f, %f, %f\ndt: %f\n",
                                               imu.lia_x, imu.lia_y, imu.lia_z,
                                               imu.acc_x, imu.acc_y, imu.acc_z,
                                               imu.vel_x, imu.vel_y, imu.vel_z,
                                               dt));
                
                waitForNextHardwareCycle();
            }
            
            log_stream.write(log.getBytes());
            log_stream.close();
            log_file_info = logfile.toString();
        }
        catch(Exception e)
        {
            for(;;)
            {
                telemetry.addData("error initializing log file", 0);
                waitOneFullHardwareCycle();
            }
        }
        for(;;)
        {
            telemetry.addData("test complete, file saved to", log_file_info);
            waitOneFullHardwareCycle();
        }
    }
}
