package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.opmodes.IK_solver;
import com.qualcomm.ftcrobotcontroller.opmodes.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import android.content.Context;
import android.app.Activity;
import android.view.View;
import android.widget.SeekBar;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;

public class Mk2Teleop extends LinearOpMode
{
    public static final float encoder_ticks_per_radian = 1440.0f/(2.0f*(float)Math.PI); //TODO: might want to make this a global const
    public static final float potentiometer_range = 333.33333333333333333333333333333333333f;
    public static final float threshold = 0.1f;

    DeviceInterfaceModule dim;
    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor shoulder;
    DcMotor elbow;
    DcMotor intake;
    
    Servo hand_servo;
    Servo intake_servo;
    
    public Mk2Teleop() {}

    float lerp(float a, float b, float t)
    {
        if(t > 1.0f) return b;
        if(t < 0.0f) return a;
        return a+(b-a)*t;
    }
    
    //in case the built in PID control in the ModernRobotics controllers doesn't work
    //TODO: can probably beat PID by actually solving the arms equations of motion (feedforward)
    class PIDController
    {
        public float k_p;
        public float k_i;
        public float k_d;
        public float k_d2;
        
        public float i = 0.0f;
        public float p_old;
        public float d = 0.0f;
        public float p2_old;
        public float d2 = 0.0f;
            
        public PIDController(float K_P, float K_I, float K_D, float K_D2, float initial_val, float initial_val2)
        {
            k_p = K_P;
            k_i = K_I;
            k_d = K_D;
            k_d2 = K_D2;
            p_old = initial_val;
            p2_old = initial_val2;
        }
        
        float getControl(float p, float p2, float dt)
        {
            d2 = lerp((p2-p2_old)/dt,
                      d2,
                      (float)Math.exp(-20.0*dt));
            p2_old = p2;

            if(i != i) i = 0.0f; //this will trigger if i is NaN
            i += p*dt; //might want to try different integrators
            d = lerp((p-p_old)/dt,
                     d,
                     (float)Math.exp(-20.0*dt));
            p_old = p;
            return k_p*p+k_i*i;//+k_d*d+k_d2*d2;
        }
    }
    
    //NOTE: these vector functions are 2d only
    static float[] scale(float[] v, float s)
    {
        float[] out = new float[2]; //TODO: 99% of the time this will is an uneccessary allocation
        out[0] = v[0]*s;
        out[1] = v[1]*s;
        return out;
    }
    
    static float sq(float a)
    {
        return a*a;
    }
    
    static float invSqrt(float a)
    {
        int ai = Float.floatToRawIntBits(a);
        int isqrti;
        float isqrt;
        isqrti = 0x5f375a86 - (ai >> 1); //magic
        isqrt = Float.intBitsToFloat(isqrti);
        isqrt *= (1.5f - (a*0.5f*isqrt*isqrt));
        isqrt *= (1.5f - (a*0.5f*isqrt*isqrt)); //second itteration, uncomment if greater accuracy is needed
        return isqrt;
    }
    
    static void normalizeScale(float[] v, float s)
    {
        float normsq = v[0]*v[0]+v[1]*v[1];
        v[0] *= invSqrt(normsq)*s;
        v[1] *= invSqrt(normsq)*s;
    }
    
    static float deadZone(float val)
    {
        if(Math.abs(val) < threshold)
        {
            val = 0.0f;
        }
        else
        {
            val = val-(val/Math.abs(val))*threshold;
            val /= 1.0f-threshold;
        }
        return val;
    }
    
    static void deadZone(float[] stick)
    {
        float norm = (float)Math.sqrt(stick[0]*stick[0]+stick[1]*stick[1]);
        if(norm < threshold)
        {
            stick[0] = 0;
            stick[1] = 0;
        }
        else stick = scale(stick, ((norm-threshold)/(1.0f-threshold))/norm);
    }
    
    static void squareDeadZone(float[] stick)
    {
        if(Math.abs(stick[0]) < threshold)
        {
            stick[0] = 0;
        }
        if(Math.abs(stick[1]) < threshold)
        {
            stick[1] = 0;
        }
    }
    
    static void add(float[] a, float[] b)
    {
        a[0] += b[0];
        a[1] += b[1];
    }
    static final float P_0 = 0.0f;
    static final float P_1 = FtcRobotControllerActivity.slider_0/100.0f;
    static final float P_2 = 1.0f;
    
    static float qBezier(float t){
        return (t >= 0? 1:-1)*((1-t)*((1-t)*P_0 + t*P_1) + t*((1-t)*P_1 + t*P_2));
    }
    
    @Override public void runOpMode()
        throws InterruptedException
    {
        int elbow_potentiometer_port = 7;
        dim = hardwareMap.deviceInterfaceModule.get("dim");

        left_drive  = hardwareMap.dcMotor.get("left_d");
        right_drive = hardwareMap.dcMotor.get("right_d");
        shoulder    = hardwareMap.dcMotor.get("shoulder");
        elbow       = hardwareMap.dcMotor.get("elbow");
        intake      = hardwareMap.dcMotor.get("intake");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        shoulder.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        elbow.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        
        /* shoulder.setPower(1.0); */
        
        hand_servo = hardwareMap.servo.get("servo_1");
        intake_servo = hardwareMap.servo.get("s");
        
        waitForStart();
        
        Button joystick1 = new Button();
        Button joystick2 = new Button();
        
        float elbow_potentiometer_angle = 0.0f;
        float elbow_encoder_pos = 0.0f;
        
        boolean arm_mode = false;
        
        float hand_servo_position = 0.0f;
        
        /*try
        {
            byte[] joystick1 = gamepad1.toByteArray();
        } catch (RobotCoreException e)
        {
            e.printStackTrace();
        }*/
        float shoulder_position = 0.0f;
        double old_time = time-0.005;
        for(;;)
        {
            double new_time = time;
            double dt = new_time-old_time;
            old_time = new_time;
            
            //================================= Control Map =====================================
            //drive
            float[] drive_stick = new float[]{-gamepad1.left_stick_x, -gamepad1.left_stick_y};

            telemetry.addData("P_1", P_1);
            //arm
                        
            float shoulder_pullup_control = gamepad2.left_stick_y;
            float winch_pullup_control = -gamepad2.right_stick_y;
            
            //intake
            boolean intakeOn = joystick1.toggle(Button.Buttons.RIGHT_BUMPER);
            boolean intakeReverse = joystick1.toggle(Button.Buttons.LEFT_BUMPER);
            //hand
            float stick_h1 = -gamepad1.right_stick_x;
            float stick_h2 = -gamepad2.right_stick_y;
            float hand_servo_control = -gamepad1.right_stick_y;
            
            //===================================================================================
            
            //drive
            //TODO: Add traction control -> Need IMU integration and encoder set-up
            deadZone(drive_stick);
            float left_power = drive_stick[1]-drive_stick[0];
            float right_power = drive_stick[1]+drive_stick[0];
            right_power = qBezier(right_power);
            left_power = qBezier(left_power);
            right_power = Range.clip(right_power, -1, 1);
            left_power = Range.clip(left_power, -1, 1);
            right_drive.setPower(right_power);
            left_drive.setPower(left_power);
            
            //arm
            //manual arm control
            float shoulder_power = shoulder_pullup_control;
            float winch_power = winch_pullup_control;
            winch_power = Range.clip(winch_power, -1, 1);
            //shoulder_position += 3.0*shoulder_power*dt;
            //shoulder_position = Range.clip(shoulder_position, -1, 1);
            
            //shoulder.setTargetPosition((int)(encoder_ticks_per_radian*shoulder_position));
            shoulder_power = Range.clip(shoulder_power, -1, 1);
            shoulder.setPower(shoulder_power);
            elbow.setPower(winch_power);                

            
            //hand
            float h1_power = deadZone(stick_h1);
            /* float h2_power = deadZone(stick_h2); */
            
            //rotateHandServo(h1_power, 0, (float)dt);
            /* rotateHandServo(h2_power, 1, dt); */
            
            hand_servo_position += hand_servo_control*dt;            
            hand_servo_position = (float)Range.clip(hand_servo_position, 0.0, 1.0);
            hand_servo.setPosition(hand_servo_position);
            
            /* //intake */
            intake_servo.setPosition(intakeOn ? 0.5+(0.5 * (intakeReverse ? -1 : 1)) : 0.5);
            
            telemetry.addData("delta t", String.format("%.2f", dt*1000.0f) + "ms");
            telemetry.addData("shoulder power", String.format("%.2f", shoulder_power));
            
            //Refreshes
            try
            {
                joystick1.updateButtons(gamepad1.toByteArray());
                joystick2.updateButtons(gamepad2.toByteArray());
            }
            catch (RobotCoreException e)
            {
                e.printStackTrace();
            }
            waitOneFullHardwareCycle();
        }
    }
}
