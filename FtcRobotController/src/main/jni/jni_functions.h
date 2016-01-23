#ifndef JNI_FUNCTIONS
#define JNI_FUNCTIONS

#include "misc.h"

#include <jni.h>
#include <stdlib.h>
#include <setjmp.h>

jmethodID waitForStartID;
jmethodID waitOneFullHardwareCycleID;
jmethodID waitForNextHardwareCycleID;
jmethodID robotStateInID;
jmethodID robotStateOutID;
jmethodID applyRobotStateID;


#define waitForStart() env->CallVoidMethod(self, waitForStartID)
#define waitOneFullHardwareCycle() env->CallVoidMethod(self, waitOneFullHardwareCycleID)
#define waitForNextHardwareCycle() env->CallVoidMethod(self, waitForNextHardwareCycleID)
#define robotStateIn() env->CallVoidMethod(self, robotStateInID)
#define robotStateOut() env->CallVoidMethod(self, robotStateOutID)
#define applyRobotState() env->CallVoidMethod(self, applyRobotStateID)

jbyteArray jrobot_state;

struct RobotState
{
    byte * state;
    bool8 is_copy;
};

RobotState robot_state;

JNIEnv * env;
jobject self;

void initJNI()
{
    jclass cls = env->GetObjectClass(self);

    //functions
    waitForStartID = env->GetMethodID(cls, "waitForStart", "()V");

    waitOneFullHardwareCycleID = env->GetMethodID(cls, "waitOneFullHardwareCycle", "()V");
    waitForNextHardwareCycleID = env->GetMethodID(cls, "waitForNextHardwareCycle", "()V");

    robotStateInID = env->GetMethodID(cls, "robotStateIn", "()V");
    robotStateOutID = env->GetMethodID(cls, "robotStateOut", "()V");

    //setup pinned array
    jfieldID jrobot_stateID = env->GetFieldID(cls, "robot_state", "[B");
    jrobot_state = (jbyteArray) env->GetObjectField(self, jrobot_stateID);
    /* if(env->GetArrayLength(jrobot_state) != rsid_size) */
    /* { */
    /*     //TODO: give error */
    /* } */

    robot_state.state = (byte *) env->GetByteArrayElements(jrobot_state, &robot_state.is_copy);
    assert(robot_state.state);
    if(robot_state.is_copy)
    {
        //TODO: give warning that the GC did not pin the array and perfomance may be impacted
    }
}

void cleanupJNI();

static jmp_buf exit_jump;

void updateRobot()
{
    env->ReleaseByteArrayElements(jrobot_state, (jbyte *) robot_state.state, JNI_COMMIT);
    robotStateOut();
    waitForNextHardwareCycle();
    if(env->ExceptionOccurred() != 0)
    {
        cleanupJNI();
        longjmp(exit_jump, 1);
    }
    robotStateIn();
}

void cleanupJNI()
{
    env->ReleaseByteArrayElements(jrobot_state, (jbyte *) robot_state.state, 0);
}

#endif
