package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.opmodes.IK_solver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Mk2Teleop extends LinearOpMode
{
    static final float encoder_ticks_per_radian = 1440.0f/2.0f*((float)Math.PI); //TODO: might want to make this a global const
    static final float threshold = 0.1f;
    
    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor shoulder;
    DcMotor elbow;
    DcMotor intake;
    
    Servo[] hand = new Servo[2];
    float[] hand_positions = new float[]{0.0f, 0.0f};
    
    float[] arm_pos_target = new float[]{0.25f, 0.25f};
    float[] arm_motor_targets = IK_solver.getArmTargets(arm_pos_target);
    
    public Mk2Teleop() {}
    
    //in case the built in PID control in the ModernRobotics controllers doesn't work
    //TODO: can probably beat PID by actually solving the arms equations of motion (feedforward)
    class PIDController
    {
        public final float k_p;
        public final float k_i;
        public final float k_d;
        
        public float i = 0.0f;
        public float p_old;
        
        public PIDController(float p, float i, float d, float initial_val){k_p = p; k_i = i; k_d = d; p_old = initial_val;}
        
        float getControl(float p, float dt)
        {
            i += p*dt; //might want to try different integrators
            float d = (p-p_old)/dt; //might need to put this through a filter
            return k_p*p+k_i*i+k_d*d;
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
    
    static float[] add(float[] a, float[] b)
    {
        a[0] += b[0];
        a[1] += b[1];
        return a;
    }
    
    @Override public void runOpMode()
        throws InterruptedException
    {
        left_drive  = hardwareMap.dcMotor.get("left_d");
        right_drive = hardwareMap.dcMotor.get("right_d");
        shoulder    = hardwareMap.dcMotor.get("shoulder");
        elbow       = hardwareMap.dcMotor.get("elbow");
        intake      = hardwareMap.dcMotor.get("intake");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        elbow.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        
        //hand[0] = hardwareMap.servo.get("servo_1");
        //hand[1] = hardwareMap.servo.get("servo_2");
        
        //PIDController shoulder_pid = new PIDController(1.0f, 0.0f, 0.4f, arm_motor_targets[0]);
        //PIDController elbow_pid = new PIDController(1.0f, 0.0f, 0.4f, arm_motor_targets[1]);
        
        waitForStart();
        
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        
        float dt;
        float new_time = 0;
        float old_time = 0;
        
        boolean intake_on = false;
        boolean prev_intake_toggle = false;
        
        boolean pullup_mode = false;
        boolean prev_pullup_mode_toggle = false;
        
        for(;;)
        {
            //////////control mapping////////////////
            //TODO: gamepad wrapper or helper library for easy toggle support and such
            //drive
            float[] drive_stick = new float[]{-gamepad1.left_stick_x, -gamepad1.left_stick_y};
            
            //arm
            float[] arm_stick = new float[]{-gamepad1.right_stick_x, -gamepad1.right_stick_y};
            
            boolean pullup_mode_toggle = gamepad1.a;
            
            float shoulder_pullup_control = gamepad2.left_stick_y;
            float winch_pullup_control = gamepad2.right_stick_y;
            
            //intake
            boolean intake_toggle = gamepad1.right_bumper;
            
            //hand
            float stick_h1 = -gamepad2.right_stick_x;
            float stick_h2 = -gamepad2.right_stick_y;
            
            /////////end control mapping//////////////
            
            new_time = (float) timer.time();
            dt = new_time-old_time;
            old_time = new_time;

            //drive
            deadZone(drive_stick);
            float left_power = drive_stick[1]-drive_stick[0];
            float right_power = drive_stick[1]+drive_stick[0];
            right_power = Range.clip(right_power, -1, 1);
            left_power = Range.clip(left_power, -1, 1);
            right_drive.setPower(right_power);
            left_drive.setPower(left_power);

            //arm
            if(pullup_mode_toggle && !prev_pullup_mode_toggle)
            {
                pullup_mode = !pullup_mode;
                if(pullup_mode)
                {
                    shoulder.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    elbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                }
                else
                {
                    shoulder.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    elbow.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    //TODO: set arm position to wherever it is when the switch occurs
                }
            }
            prev_pullup_mode_toggle = pullup_mode_toggle;
            
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
                deadZone(arm_stick);
            
                add(arm_pos_target, scale(arm_stick, 0.1f*dt));
                arm_motor_targets = IK_solver.getArmTargets(arm_pos_target);
            
                //manual PID
                /* float shoulder_power = shoulder_pid.getControl(encoder_ticks_per_radian*arm_motor_targets[0]
                /*                                                -shoulder.getCurrentPosition(), dt);
                /* float elbow_power = (elbow_pid.getControl(encoder_ticks_per_radian*arm_motor_targets[1]
                /*                                           -elbow.getCurrentPosition(), dt)); */            
                
                shoulder.setTargetPosition((int)(encoder_ticks_per_radian*arm_motor_targets[0]));
                elbow.setTargetPosition((int)(encoder_ticks_per_radian*arm_motor_targets[1]));
            }
            
            //hand
            float h1_power = deadZone(stick_h1);
            float h2_power = deadZone(stick_h2);
            
            rotateHandServo(h1_power, 0, dt);
            rotateHandServo(h2_power, 1, dt);
            
            //intake
            if(intake_toggle && !prev_intake_toggle)
            {
                intake_on = !intake_on;
            }
            prev_intake_toggle = intake_toggle;
            intake.setPower(intake_on ? 1.0: 0.0);
            
            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("delta t", String.format("%.2f", dt) + "ms");
            telemetry.addData("Shoulder stick", "shoulder val:" + String.format("%.2f", arm_stick[0]));
            telemetry.addData("elbow power", "elbow pwr: " + String.format("%.2f", arm_stick[1]));
            
            waitOneFullHardwareCycle();
        }
    }
}
