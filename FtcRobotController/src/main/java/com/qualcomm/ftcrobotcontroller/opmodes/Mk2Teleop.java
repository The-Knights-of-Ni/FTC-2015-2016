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
    
    /* DcMotor left_drive; */
    /* DcMotor right_drive; */
    DcMotor shoulder;
    DcMotor elbow;
    /* DcMotor intake; */
    
    Servo[] hand = new Servo[2];
    float[] hand_positions = new float[]{0.0f, 0.0f};
    
    float[] arm_pos_target = new float[]{15.0f, (float)Math.PI/4.0f};//new float[]{(float)Math.PI/2.0f, (float)Math.PI/2.0f};//new float[]{13.5f, 15.0f};
    float[] arm_motor_targets = IK_solver.getArmTargetsPolar(arm_pos_target, false);
    
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
    
    void rotateContinuousHandServo(float omega, int servo, float dt)
    {
        omega = Range.clip(omega, -1.0f, 1.0f);
        //hand[servo].setPosition(omega);
    }
    
    void rotateHandServo(float omega, int servo, float dt)
    {
        hand_positions[servo] += omega*dt;
        hand_positions[servo] = Range.clip(hand_positions[servo], -1.0f, 1.0f);
        //hand[servo].setPosition(hand_positions[servo]);
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
    
    @Override public void runOpMode()
        throws InterruptedException
    {
        int elbow_potentiometer_port = 7;
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        
        //left_drive  = hardwareMap.dcMotor.get("left_d");
        //right_drive = hardwareMap.dcMotor.get("right_d");
        shoulder    = hardwareMap.dcMotor.get("shoulder");
        elbow       = hardwareMap.dcMotor.get("elbow");
        //intake      = hardwareMap.dcMotor.get("intake");
        //right_drive.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        shoulder.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        //elbow.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        
        //hand[0] = hardwareMap.servo.get("servo_1");
        //hand[1] = hardwareMap.servo.get("servo_2");
        
        float k_p_winch = 0.77f;
        float k_i_winch = 0.08f;
        float k_d_winch = 0.0f;
        float k_p_pulley = 1.0f;
        float k_i_pulley = 0.5f;
        float k_d_pulley = 0.0f;
        
        PIDController elbow_pid = new PIDController(k_p_winch, k_i_winch, k_d_winch, 0.0f, arm_motor_targets[1], 0.0f);
        
        waitForStart();
        
        
        Button joystick1 = new Button();
        Button joystick2 = new Button();
        
        float elbow_potentiometer_angle = 0.0f;
        float elbow_encoder_pos = 0.0f;

        boolean arm_mode = false;
        
        /*try
        {
            byte[] joystick1 = gamepad1.toByteArray();
        } catch (RobotCoreException e)
        {
            e.printStackTrace();
        }*/
        double old_time = time-0.005;
        for(;;)
        {
            double new_time = time;
            double dt = new_time-old_time;
            old_time = new_time;
            
            //================================= Control Map =====================================
            //drive
            float[] drive_stick = new float[]{-gamepad1.left_stick_x, -gamepad1.left_stick_y};
            
            //arm
            
            boolean winch_mode = joystick2.singlePress(Button.Buttons.RIGHT_BUMPER);
            boolean pulley_mode = joystick2.singlePress(Button.Buttons.LEFT_BUMPER);
            float[] arm_stick = new float[]{
                -gamepad2.right_stick_y,
                (-gamepad2.left_stick_y*(float)Math.cos(arm_pos_target[1])
                      -gamepad2.left_stick_x*(float)Math.sin(arm_pos_target[1]))};

            float[] arm_rectangular_stick = {gamepad2.right_stick_x,                 
                                               -gamepad2.right_stick_y};
            
            telemetry.addData("arm tangent vector", String.format("(%.2f, %.2f)",
                                                                  -(float)Math.cos(arm_pos_target[1]),
                                                                  -(float)Math.sin(arm_pos_target[1])));
            
            boolean pullup_mode = joystick2.toggle(Button.Buttons.A);
            
            float shoulder_pullup_control = gamepad2.left_stick_y;
            float winch_pullup_control = gamepad2.right_stick_y;
            
            //intake
            boolean intakeOn = joystick1.toggle(Button.Buttons.RIGHT_BUMPER);
            boolean intakeReverse = joystick1.toggle(Button.Buttons.LEFT_BUMPER);
            //hand
            float stick_h1 = -gamepad2.right_stick_x;
            float stick_h2 = -gamepad2.right_stick_y;
            
            //===================================================================================
            
            //drive
            //TODO: Add traction control -> Need IMU integration and encoder set-up
            /* deadZone(drive_stick); */
            /* float left_power = drive_stick[1]-drive_stick[0]; */
            /* float right_power = drive_stick[1]+drive_stick[0]; */
            /* right_power = Range.clip(right_power, -1, 1); */
            /* left_power = Range.clip(left_power, -1, 1); */
            /* right_drive.setPower(right_power); */
            /* left_drive.setPower(left_power); */
            
            //arm
            if(winch_mode)
            {
                arm_mode = false;
                arm_pos_target[0] = 27.0f;
                arm_pos_target[1] = (float)Math.PI/4;
                elbow_pid.i = 0.0f;
            }
            else if(pulley_mode)
            {
                arm_mode = true;
                arm_pos_target[0] = -19.0f;
                arm_pos_target[1] = 15.0f;
                elbow_pid.i = 0.0f;
            }
            
            if(pullup_mode)
            {
                shoulder.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                elbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            }
            else
            {
                shoulder.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1.0);
                elbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                //TODO: set arm position to wherever it is when the switch occurs
            }
            
            if(pullup_mode)
            {
                //manual arm control
                float shoulder_power = shoulder_pullup_control;
                float winch_power = winch_pullup_control;
                winch_power = Range.clip(winch_power, -1, 1);
                shoulder_power = Range.clip(shoulder_power, -1, 1);
                shoulder.setPower(shoulder_power);
                elbow.setPower(winch_power);                
            }
            else
            {
                //IK arm control
                //TODO: gyro/compass stabalization, feedforward control(might not need it until after ndk transition)
                elbow_potentiometer_angle = lerp(
                    (360.0f-(180.0f-potentiometer_range*0.5f+potentiometer_range*(((float)dim.getAnalogInputValue(elbow_potentiometer_port))/(1023.0f)))-12.0f)*(float)Math.PI/180.0f,
                    elbow_potentiometer_angle,
                    (float)Math.exp(-20.0*dt));
                
                if(arm_mode)
                {
                    deadZone(arm_rectangular_stick);
                    add(arm_pos_target, scale(arm_rectangular_stick, 10.0f*(float) dt));
                    arm_motor_targets = IK_solver.getArmTargetsRectangular(arm_pos_target, arm_mode);
                    
                    elbow_pid.k_p = k_p_pulley;
                    elbow_pid.k_i = k_i_pulley;
                    elbow_pid.k_d = k_d_pulley;
                }
                else
                {
                    squareDeadZone(arm_stick);
                    
                    arm_stick[0] *= 10.0f;
                    arm_stick[1] *= 2.0f;
                    add(arm_pos_target, scale(arm_stick, (float) dt));
                    /* if(arm_mode) */
                    /* { */
                    /*     float y = arm_pos_target[0]*(float)Math.sin(arm_pos_target[1]); */
                    /*     float cosine = ((float)Math.cos(arm_pos_target[1])); */
                    /*     float new_x = 10.0f*arm_x_control*(float)dt+arm_pos_target[0]*((float)Math.cos(arm_pos_target[1])); */
                    /*     arm_pos_target[0] = (float)Math.sqrt(new_x*new_x+y*y); */
                    /*     arm_pos_target[1] = (float)Math.atan2(y, new_x); */
                    /* } */

                    arm_motor_targets = IK_solver.getArmTargetsPolar(arm_pos_target, arm_mode);
                    
                    elbow_pid.k_p = k_p_winch;
                    elbow_pid.k_i = k_i_winch;
                    elbow_pid.k_d = k_d_winch;
                }
                
                /* k_p_winch = FtcRobotControllerActivity.slider_0/100.0f; */
                /* k_i_winch = FtcRobotControllerActivity.slider_1/100.0f; */
                /* k_p_pulley = FtcRobotControllerActivity.slider_2/100.0f; */
                /* k_i_pulley = FtcRobotControllerActivity.slider_3/100.0f; */
                
                /* elbow_pid.k_p = FtcRobotControllerActivity.slider_0/100.0f; */
                /* elbow_pid.k_i = FtcRobotControllerActivity.slider_1/100.0f; */
                /* elbow_pid.k_d = FtcRobotControllerActivity.slider_2/100.0f; */
                /* elbow_pid.k_d2 = FtcRobotControllerActivity.slider_3/100.0f; */
                
                float winch_power = elbow_pid.getControl((arm_motor_targets[1]
                                                          -elbow_potentiometer_angle),
                                                         (float)elbow.getCurrentPosition()/encoder_ticks_per_radian,
                                                         (float)dt);
                winch_power = Range.clip(winch_power, -1.0f, 0.5f);
                if(arm_mode && winch_power > 0.0f &&
                   arm_motor_targets[1]-elbow_potentiometer_angle > 0.1f &&
                   elbow_potentiometer_angle < (float)Math.PI/2) winch_power = 0.0f;
                elbow.setPower(winch_power);
                telemetry.addData("winch_power", String.format("%.2f", winch_power));
                //might need different PID constants for winch mode and pulley mode
                
                shoulder.setTargetPosition((int)(encoder_ticks_per_radian*2.0f/* *60.0f/40.0f */*(arm_motor_targets[0]-Math.PI/2)));
            }
            
            //hand
            /* float h1_power = deadZone(stick_h1); */
            /* float h2_power = deadZone(stick_h2); */
            
            /* rotateHandServo(h1_power, 0, dt); */
            /* rotateHandServo(h2_power, 1, dt); */
            
            /* //intake */
            /* intake.setPower(intakeOn ? 1.0 * (intakeReverse ? -1 : 1) : 0.0); */
            
            telemetry.addData("target pos", String.format("(%.2f, %.2f)", arm_pos_target[0], arm_pos_target[1]));
            telemetry.addData("delta t", String.format("%.2f", dt*1000.0f) + "ms");
            telemetry.addData("elbow potentiometer", String.format("%.2f", elbow_potentiometer_angle*180.0f/(float)Math.PI));
            telemetry.addData("r stick", String.format("%.2f", arm_stick[0]));
            telemetry.addData("theta stick", String.format("%.2f", arm_stick[1]));
            telemetry.addData("shoulder target", String.format("%.2f", arm_motor_targets[0]*180.0f/(float)Math.PI));
            telemetry.addData("elbow target", String.format("%.2f", arm_motor_targets[1]*180.0f/(float)Math.PI));
            telemetry.addData("slider 0", String.format("%d", FtcRobotControllerActivity.slider_0));
            telemetry.addData("slider 1", String.format("%d", FtcRobotControllerActivity.slider_1));
            telemetry.addData("slider 2", String.format("%d", FtcRobotControllerActivity.slider_2));
            telemetry.addData("slider 3", String.format("%d", FtcRobotControllerActivity.slider_3));
            telemetry.addData("slider 4", String.format("%d", FtcRobotControllerActivity.slider_4));
            
            telemetry.addData("k_p", String.format("%.2f", elbow_pid.k_p));
            telemetry.addData("k_i", String.format("%.2f", elbow_pid.k_i));
            telemetry.addData("k_d", String.format("%.2f", elbow_pid.k_d));
            
            //Refreshes
            try
            {
                joystick1.updateButtons(gamepad1.toByteArray());
                joystick2.updateButtons(gamepad2.toByteArray());
            } catch (RobotCoreException e)
            {
                e.printStackTrace();
            }
            waitOneFullHardwareCycle();
        }
    }
}
