package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftcrobotcontroller.opmodes.Mk3AutoRobotStateElements;

import java.nio.ByteBuffer;

import android.hardware.Camera;


public class Mk3Auto extends LinearOpMode {
    Camera camera = null;
    int camera_w = 0;
    int camera_h = 0;
    CameraPreviewCallback camera_preview_callback;
    
    byte[] camera_buffer = null;
    
    class CameraPreviewCallback implements Camera.PreviewCallback
    {
        
        
        CameraPreviewCallback(){}
        public void onPreviewFrame(byte[] data, Camera camera)
        {
            camera.addCallbackBuffer(camera_buffer);
        }
    }
    
    /* Start NDK Stuff*/
    byte[] robot_state;
    int elbow_potentiometer_port = 7;
    
    public Mk3Auto() {
        camera = FtcRobotControllerActivity.camera;
        Camera.Parameters parameters = camera.getParameters();
        Camera.Size camera_size = parameters.getPreviewSize();
        camera_w = camera_size.width;
        camera_h = camera_size.height;
        
        robot_state = new byte[Mk3AutoRobotStateElements.robot_state_size];
        Mk3AutoRobotStateElements.robot_state = robot_state;
        
        Mk3AutoRobotStateElements.set_camera_w(camera_w);
        Mk3AutoRobotStateElements.set_camera_h(camera_h);
        
        camera_buffer = new byte[camera_w*camera_h*4];
        camera_preview_callback = new CameraPreviewCallback();
        
        camera.setPreviewCallbackWithBuffer(camera_preview_callback);
        camera.addCallbackBuffer(camera_buffer);
    }
    
    native void main();
    
    static {
        System.loadLibrary("Mk3Auto");
    }
    
    void robotStateIn()
    {
        /*DATA IN (from electronics&driver station)*/
        //Sensors
        Mk3AutoRobotStateElements.set_time(time);
        Mk3AutoRobotStateElements.set_right_drive_encoder(right_drive.getCurrentPosition());
        Mk3AutoRobotStateElements.set_left_drive_encoder(left_drive.getCurrentPosition());
        Mk3AutoRobotStateElements.set_winch_encoder(winch.getCurrentPosition());
        Mk3AutoRobotStateElements.set_shoulder_encoder(shoulder.getCurrentPosition());
        Mk3AutoRobotStateElements.set_elbow_potentiometer(dim.getAnalogInputValue(elbow_potentiometer_port));
        Mk3AutoRobotStateElements.set_color((FtcRobotControllerActivity.red ? 1 : 0));
        if(imu.checkForUpdate()) {
            Mk3AutoRobotStateElements.set_imu_heading(imu.eul_x);
            Mk3AutoRobotStateElements.set_imu_tilt(imu.eul_y);
            Mk3AutoRobotStateElements.set_imu_roll(imu.eul_z);
            Mk3AutoRobotStateElements.set_imu_velocity_x(imu.vel_x);
            Mk3AutoRobotStateElements.set_imu_velocity_y(imu.vel_y);
            Mk3AutoRobotStateElements.set_imu_velocity_z(imu.vel_z);
        }
        
    }
    
    void robotStateOut()
    {
        /*DATA OUT (to electronics&driver station)*/
        left_drive.setPower(Mk3AutoRobotStateElements.get_left_drive());
        right_drive.setPower(Mk3AutoRobotStateElements.get_right_drive());
        winch.setPower(Mk3AutoRobotStateElements.get_winch());
        shoulder.setPower(Mk3AutoRobotStateElements.get_shoulder());
        intake.setPower(Mk3AutoRobotStateElements.get_intake());
        hand_servo.setPosition(Mk3AutoRobotStateElements.get_hand());
        slide_servo.setPosition(Mk3AutoRobotStateElements.get_slide());
        telemetry.addData("Indicator:", Mk3AutoRobotStateElements.get_indicator());
        telemetry.addData("beacon right:", (Mk3AutoRobotStateElements.get_beacon_right() == 1 ? "red" : "blue"));
        telemetry.addData("heading:", Mk3AutoRobotStateElements.get_imu_heading());
    }
    
    /* End NDK Stuff*/
    
    
    /* Start Motor Definitions */
    DeviceInterfaceModule dim;
    IMU imu;
    
    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor shoulder;
    DcMotor winch;
    DcMotor intake;
    
    Servo hand_servo;
    Servo slide_servo;
    /* End Motor Definitions*/
    
    public int updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.
    {
        ByteBuffer stick = ByteBuffer.allocate(45);
        stick.put(joystick);
        return stick.getInt(40);//Offset value
    }

    @Override public void runOpMode() throws InterruptedException {
        int elbow_potentiometer_port = 7;
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        
        I2cDevice imu_i2c_device = hardwareMap.i2cDevice.get("imu");
        imu = new IMU(imu_i2c_device);
        int error = imu.init(IMU.mode_ndof,
                (byte) (IMU.units_acc_m_per_s2 |
                        IMU.units_angle_deg |
                        IMU.units_angular_vel_deg_per_s |
                        IMU.units_temp_C |
                        IMU.units_pitch_convention_android));
        if (error != 0) {
            for (; ; ) {
                telemetry.addData("error initializing imu", 0);
                waitOneFullHardwareCycle();
            }
        }
        imu.vel_x = 0.0f;
        imu.vel_y = 0.0f;
        imu.vel_z = 0.0f;
        
        left_drive = hardwareMap.dcMotor.get("leftd");
        right_drive = hardwareMap.dcMotor.get("rightd");
        shoulder = hardwareMap.dcMotor.get("shoulder");
        winch = hardwareMap.dcMotor.get("winch");
        intake = hardwareMap.dcMotor.get("intake");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        shoulder.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        winch.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        winch.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        
        hand_servo = hardwareMap.servo.get("hand");
        slide_servo = hardwareMap.servo.get("slide");
        while (!FtcRobotControllerActivity.aligned || (!FtcRobotControllerActivity.red && !FtcRobotControllerActivity.blue))
        {
            telemetry.addData("unchecked boxes", "fix it");
            waitForNextHardwareCycle();
        }
        waitForStart();
        imu.rezero(); //Make sure you call rezero before starting, it resets the velocity integration timers and values
        
        main();
    }
}
