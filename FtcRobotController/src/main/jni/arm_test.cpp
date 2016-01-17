/*
  robot_state_elements
  {
  gamepad{float joystick1_x, float joystick1_y, float joystick2_x, float joystick2_y, float left_trigger, float right_trigger, int buttons};

  double time;
  
  float arm_shoulder_power;
  float arm_winch_power;
  
  gamepad gamepad1;
  gamepad gamepad2;
  
  float shoulder_encoder;
  float elbow_potentiometer;
  float winch_encoder;
  }
*/

#include "maths.h"
#include "Button.h"
#include "arm_test_robot_state_elements.h"
//TODO: add include guards to generator so arm_test_robot_state_elements.h can be included in arm.h
#include "arm.h"

arm_state s = {};
arm_state stable_s = {};
bool8 score_mode = true;

extern "C"
void JNI_main(JNIEnv * env, jobject self)
{
    initJNI(env, self);
    
    waitForStart();
    
    score_mode = false;
    stable_s.shoulder_theta = 2.75;
    stable_s.forearm_theta = pi;
    stable_s.winch_theta = 5.5;
    
    //TODO: make this run in a seperate thread, so it can run in a
    //      loop without the init code running every frame
    float dt;
    float old_time = time;
    do
    {
        //NOTE: time will only change when updateRobot() is called
        dt = time-old_time;
        old_time = time;
        
        ////////////////Control Map////////////////
        bool8 intake_mode_button = gamepad1.left_trigger > 0.5;
        bool8 score_mode_button = gamepad1.right_trigger > 0.5;
        
        #if 0
        arm_shoulder_power = gamepad2.left_stick.y;
        arm_winch_power = gamepad2.right_stick.y;
        #else            
        v2f target_velocity = gamepad2.joystick2*40.0;
        #endif
        ///////////////////////////////////////////

        float new_shoulder_theta = shoulder_encoder+pi*4.0/5.0;
        float new_forearm_theta = elbow_potentiometer+shoulder_encoder-pi;
        float new_winch_theta = winch_encoder;
        s.shoulder_omega = lerp((new_shoulder_theta-s.shoulder_theta)/dt, s.shoulder_omega, exp(-0.1*dt));
        s.forearm_omega = lerp((new_forearm_theta-s.forearm_theta)/dt, s.forearm_omega, exp(-0.1*dt));
        s.winch_omega = lerp((new_winch_theta-s.winch_theta)/dt, s.winch_omega, exp(-0.1*dt));
        
        s.shoulder_theta = new_shoulder_theta;
        s.forearm_theta = new_forearm_theta;
        s.winch_theta = new_winch_theta;
        
        if(intake_mode_button)
        {
            score_mode = false;
            stable_s.shoulder_theta = 2.75;
            stable_s.forearm_theta = pi;
            stable_s.winch_theta = 5.5;
        }
        if(score_mode_button)
        {
            score_mode = true;
            stable_s.shoulder_theta = 0.75;
            stable_s.forearm_theta = 0;
            stable_s.winch_theta = 1;
        }
        
        if(normSq(target_velocity) > 1.0)
        {
            stable_s = s;
            armAtVelocity(arm_shoulder_power, arm_winch_power, target_velocity, s, score_mode, dt);
        }
        else
        {
            armToState(arm_shoulder_power, arm_winch_power, stable_s, s, score_mode, dt);
        }
        
        arm_shoulder_power = clamp(arm_shoulder_power, -1.0, 1.0);
        arm_winch_power = clamp(arm_winch_power, -1.0, 1.0);
    } while(updateRobot(env, self) != 0);
    cleanupJNI(env, self);
}
