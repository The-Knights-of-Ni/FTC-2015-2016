jmethodID waitForStartID;
jmethodID waitOneFullHardwareCycleID;
jmethodID applyRobotStateID;

#define waitForStart() env->CallVoidMethod(self, waitForStartID)
#define waitOneFullHardwareCycle() env->CallVoidMethod(self, waitOneFullHardwareCycleID)
#define applyRobotState() env->CallVoidMethod(self, applyRobotStateID)

jarray jrobot_state;

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
    
    applyRobotStateID = env->GetMethodID(cls, "applyRobotState", "()V");
    
    //setup pinned array
    jfieldID jrobot_stateID = env->GetFieldID(cls, "robot_state", "[B");
    jrobot_state = (jarray) env->GetObjectField(cls, jrobot_stateID);
    if(env->GetArrayLength(jrobot_state) != rsid_size)
    {
        //TODO: give error
    }
    
    robot_state.state = (byte*) env->GetPrimitiveArrayCritical(jrobot_state, &robot_state.is_copy);
    assert(robot_state.state);
    if(robot_state.is_copy)
    {
        //TODO: give warning that the GC did not pin the array and perfomance may be impacted
    }
}

void updateRobot(JNIEnv * env, jobject self)
{
    env->ReleasePrimitiveArrayCritical(jrobot_state, robot_state.state, JNI_COMMIT);
    applyRobotState();
    waitOneFullHardwareCycle();    
}

void cleanupJNI(JNIEnv * env, jobject self)
{
    env->ReleasePrimitiveArrayCritical(jrobot_state, robot_state.state, 0);
}

#define JNI_main Java_com_qualcom_ftc_robotcontroller_opmodes_NDK_test_main
