package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.ftccommon.DbgLog;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import android.content.Context;
import java.io.File;
import java.io.FileOutputStream;
import android.os.Environment;

public class rushed_autonomous extends LinearOpMode
{
    public static final float encoder_ticks_per_radian = 1440.0f/(2.0f*(float)Math.PI); //TODO: might want to make this a global const
    
    DeviceInterfaceModule dim;
    IMU imu;

    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor shoulder;
    DcMotor elbow;
    DcMotor intake;
    
    Servo hand_servo;
    
    public boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
    }
    double old_time;
    
    /* void updateRobot() */
    /* { */
    /*     double new_time = time; */
    /*     double dt = new_time-old_time; */
    /*     old_time = new_time; */
    /*     if(imu.checkForUpdate()) */
    /*     { */
    /*         update_cycle_time = new_time-last_update_time; */
    /*         last_update_time = new_time; */
                    
    /*         log = log.concat(String.format("raw: %d, %d, %d\nhpa: %f, %f, %f\nvel: %f, %f, %f\ndt: %f\n", */
    /*                                        imu.lia_x, imu.lia_y, imu.lia_z, */
    /*                                        imu.acc_x, imu.acc_y, imu.acc_z, */
    /*                                        imu.vel_x, imu.vel_y, imu.vel_z, */
    /*                                        imu.dt)); */
    /*     } */
                
    /*     int ls = dim.getAnalogInputValue(0); */
    /*     telemetry.addData("number of reads", imu.n_reads); */
    /*     telemetry.addData("limit switch", ls); */
    /*     telemetry.addData("IMU heading", ((float)imu.eul_x)/16.0f); */
    /*     telemetry.addData("acceleration x", imu.lia_x/100.0f); */
    /*     telemetry.addData("acceleration y", imu.lia_y/100.0f); */
    /*     telemetry.addData("acceleration z", imu.lia_z/100.0f); */
    /*     telemetry.addData("IMU cycle time 1", imu.dt); */
    /*     telemetry.addData("IMU cycle time 2", update_cycle_time); */
    /*     telemetry.addData("high_passed_acceleration x", imu.acc_x/100.0f); */
    /*     telemetry.addData("high_passed_acceleration y", imu.acc_y/100.0f); */
    /*     telemetry.addData("high_passed_acceleration z", imu.acc_z/100.0f); */
    /*     telemetry.addData("velocity x", imu.vel_x/100.0f); */
    /*     telemetry.addData("velocity y", imu.vel_y/100.0f); */
    /*     telemetry.addData("velocity z", imu.vel_z/100.0f); */
    /*     telemetry.addData("cycle time", dt); */
    /*     telemetry.addData("file", logfile.toString()); */
            
    /*     waitForNextHardwareCycle(); */
    /* } */
    
    void drive(float time)
    {
        
    }
    
    @Override
    public void runOpMode()
            throws InterruptedException
    {
        resetStartTime();
        
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        I2cDevice imu_i2c_device = hardwareMap.i2cDevice.get("imu");
        
        left_drive  = hardwareMap.dcMotor.get("left_d");
        right_drive = hardwareMap.dcMotor.get("right_d");
        shoulder    = hardwareMap.dcMotor.get("shoulder");
        elbow       = hardwareMap.dcMotor.get("elbow");
        intake      = hardwareMap.dcMotor.get("intake");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        elbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        
        hand_servo = hardwareMap.servo.get("servo_1");
        
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
        
        imu.vel_x = 0.0f;
        imu.vel_y = 0.0f;
        imu.vel_z = 0.0f;
        
        telemetry.addData("waiting for start, init time in seconds", time);
        
        double old_time = 0.0;
        resetStartTime();
        
        double last_update_time = 0.0;
        double update_cycle_time = 0.0;
        waitForStart();
            
        imu.rezero(); //Make sure you call rezero before starting, it resets the velocity integration timers and values
        
        for(;;)
        {
            waitOneFullHardwareCycle();
        }
    }
}
