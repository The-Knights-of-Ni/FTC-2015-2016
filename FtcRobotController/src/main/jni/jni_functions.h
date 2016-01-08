#ifndef JNI_FUNCTIONS
#define JNI_FUNCTIONS

#include <jni.h>

jmethodID waitForStartID;
jmethodID waitOneFullHardwareCycleID;
jmethodID waitForNextHardwareCycleID;
jmethodID applyRobotStateID;

#define waitForStart() env->CallVoidMethod(self, waitForStartID)
#define waitOneFullHardwareCycle() env->CallVoidMethod(self, waitOneFullHardwareCycleID)
#define waitForNextHardwareCycle() env->CallVoidMethod(self, waitForNextHardwareCycleID)
#define applyRobotState() env->CallVoidMethod(self, applyRobotStateID)

jbyteArray jrobot_state;

struct RobotState
{
    byte * state;
    bool8 is_copy;    
};

RobotState robot_state;

void initJNI(JNIEnv * env, jobject self)
{
    jclass cls = env->GetObjectClass(self);
    
    //functions
    waitForStartID = env->GetMethodID(cls, "waitForStart", "()V");

    waitOneFullHardwareCycleID = env->GetMethodID(cls, "waitOneFullHardwareCycle", "()V");
    waitForNextHardwareCycleID = env->GetMethodID(cls, "waitForNextHardwareCycle", "()V");
    
    applyRobotStateID = env->GetMethodID(cls, "applyRobotState", "()V");
    
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

jthrowable updateRobot(JNIEnv * env, jobject self)
{
    env->ReleaseByteArrayElements(jrobot_state, (jbyte *) robot_state.state, JNI_COMMIT);
    applyRobotState();
    waitForNextHardwareCycle();
    return env->ExceptionOccurred();
}

void cleanupJNI(JNIEnv * env, jobject self)
{
    env->ReleaseByteArrayElements(jrobot_state, (jbyte *) robot_state.state, 0);
}

#endif
