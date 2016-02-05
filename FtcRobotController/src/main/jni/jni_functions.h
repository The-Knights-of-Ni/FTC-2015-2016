#ifndef JNI_FUNCTIONS
#define JNI_FUNCTIONS

#include <setjmp.h>
#include <stdlib.h>

#include "misc.h"

struct RobotState
{
    byte * state;
    bool8 is_copy;
};

RobotState robot_state;

uint rsid_current = 0;

static jmp_buf exit_jump;

//interruptable blocks are exited when update robot detects an interuptted exception
//NOTE: only the most recent interuptable block will be exited, only one is intended for use
#define interruptable if(!setjmp(exit_jump))

//dummy struct and variable to operator overload on
struct jniOutStruct{};
jniOutStruct jniOutVar;

char * jni_import_string = 0;
char * jni_variables_string = 0;
char * jni_run_opmode_string = 0;
char * jni_misc_string = 0;
char * jni_constructor_string = 0;

//////////////////////////////////////////////
#ifdef GENERATE //compiling in generate mode//
//////////////////////////////////////////////
#include <string.h>
#define max_java_code 5*kilobyte

//dummy jni stuff
typedef byte jobject;
typedef byte jbyteArray;
typedef byte jclass;
typedef byte jfieldID;
typedef byte jbyte;

struct JNIEnv
{
    jfieldID GetFieldID(jclass, const char *, const char *){return 0;}
    jclass GetObjectField(jobject, jfieldID){return 0;}
    jclass GetObjectClass(jobject){return 0;}
    void * GetByteArrayElements(jbyteArray, void *){return 0;}
    int GetIntField(jclass, jfieldID){return 0;}
    void ReleaseByteArrayElements(jbyteArray, jbyte *, int){}
};

JNIEnv * env;
jobject self;

#define waitForStart()
#define waitOneFullHardwareCycle()
#define waitForNextHardwareCycle()
#define robotStateIn()
#define robotStateOut()
#define initJNI()
#define updateRobot()
#define cleanupJNI()
#define JNI_COMMIT 0

//new empty 0 terminated string
char * str_malloc(uint max_string_size)
{
    char * out = (char *) malloc(max_string_size);
    *out = 0;
    return out;
}

char * rsout_code = str_malloc(max_java_code); //TODO: expand if limit is reached
char * rsout_code_current = rsout_code;
char * rsin_code = str_malloc(max_java_code);
char * rsin_code_current = rsin_code;

#define constStrncmp(a, constant_string, max_len) (strncmp((a), (constant_string), min((max_len), sizeof(constant_string)-1)))

#define constStrcmp(a, constant_string) (strncmp((a), (constant_string), sizeof(constant_string)-1))

#define robot_state_reserved_addresses (1*kilobyte)
void * robot_state_first_address = malloc(robot_state_reserved_addresses);
//TODO: figure out a better way to do this, the memory is not actually needed, just the addresses

//TODO: remove redundent setting of rsid_current in the java file

/*TODO(maybe): give a warning if you try to set a pointer to two
  things, would require making another dummy struct and overloading =*/
//TODO: allow for variables to be used for both input and output (1/2
//done)
#define defineJniIn(type, Type)                                         \
    type * jni##Type##In_line(const char * s, const char * filename, int line) \
    {                                                                   \
        int rsid_original = rsid_current;                               \
        const char * t = s;                                             \
        for(; *t; t++)                                                  \
        {                                                               \
            if(constStrcmp(t, "return") == 0)                           \
            {                                                           \
                int n_printed = sprintf(rsin_code_current, "{\n%.*s", t-s, s); \
                assert(n_printed >= 0);                                 \
                rsin_code_current += n_printed;                         \
                t += sizeof("return");                                  \
                int n_return_chars = 0;                                 \
                for(; t[n_return_chars] != ';'; n_return_chars++)       \
                {                                                       \
                    if(t[n_return_chars] == 0)                          \
                    {                                                   \
                        printf("%s:%d: error: expected ';' after return value\n", filename, line); \
                        return 0;                                       \
                    }                                                   \
                }                                                       \
                n_printed = sprintf(rsin_code_current,                  \
                                    "set"#Type"(%d, %.*s);\n",          \
                                    rsid_current,                       \
                                    n_return_chars, t);                 \
                assert(n_printed >= 0);                                 \
                rsin_code_current += n_printed;                         \
                                                                        \
                n_printed = sprintf(rsin_code_current, "%s\n",          \
                                    t+n_return_chars+1);                \
                assert(n_printed >= 0);                                 \
                rsin_code_current += n_printed;                         \
                                                                        \
                rsid_current += sizeof(type);                           \
                                                                        \
                n_printed = sprintf(rsin_code_current, "rsid_current = %d;\n}\n", \
                                    rsid_current);                      \
                assert(n_printed >= 0);                                 \
                rsin_code_current += n_printed;                         \
                                                                        \
                break;                                                  \
            }                                                           \
        }                                                               \
        return (type*)((byte*) robot_state_first_address+rsid_original); \
    }

//TODO: arrays

//TODO: allow for non-tightly packed structs
//TODO: pull out code that is shared with jniTypeIn_line
void * jniStructIn_line(int struct_size, const char * s, const char * filename, int line)
{
    const char * t = s;
    for(; *t; t++)
    {
        if(constStrcmp(t, "return") == 0)
        {
            int n_printed = sprintf(rsin_code_current, "{\nrsid_current = %d;\n%.*s",rsid_current, t-s, s);
            assert(n_printed >= 0);
            rsin_code_current += n_printed;
            t += sizeof("return");
            while(*t++ != '{')
            {
                if(*t == 0)
                {
                    printf("%s:%d: error: expected '{' after return\n", filename, line);
                    return 0;
                }
            }
            
            int n_return_chars = 0;
            for ever
            {
                bool end_reached = false;
                int paren_scope = 0;
                for(n_return_chars = 0 ; paren_scope == 0 && t[n_return_chars] != ','; n_return_chars++)
                {
                    if(t[n_return_chars] == '(') paren_scope++;
                    if(t[n_return_chars] == ')') paren_scope--;
                    if(t[n_return_chars] == '}')
                    {
                        end_reached = true;
                        break;
                    }
                    if(t[n_return_chars] == 0)
                    {
                        printf("%s:%d: error: unexpected end\n", filename, line);
                        return 0;
                    }
                }
                /* the java compiler will figure out the type of the
                   arg in put(), the danger is that it will not give an
                   error if you put in the incorrect type, this could
                   be solved if we C++ had a way to loop over the
                   members of a struct */
                n_printed = sprintf(rsin_code_current,
                                    "setRelative(%.*s);\n",
                                    n_return_chars, t);
                assert(n_printed >= 0);
                rsin_code_current += n_printed;
                t += n_return_chars+1;
                
                if(end_reached) break;
            }
            
            n_printed = sprintf(rsin_code_current, "%s\n}\n", t);
            assert(n_printed >= 0);
            rsin_code_current += n_printed;           
            break;
        }
    }
    int rsid_original = rsid_current;
    rsid_current += struct_size;
    return ((byte*)robot_state_first_address+rsid_original);
}

jniOutStruct operator, (jniOutStruct jos, const char * s)
{
    int n_printed = sprintf(rsout_code_current, "%s", s);
    assert(n_printed >= 0);
    rsout_code_current += n_printed;
    return jos;
}

#define defineJniOut(type, Type)                                        \
    jniOutStruct operator, (jniOutStruct jos, type * & a)               \
    {                                                                   \
        int rs_index = ((byte*)a-(byte*)robot_state_first_address);     \
        if(rs_index >= 0 && rs_index < rsid_current)                    \
        {                                                               \
            int n_printed = sprintf(rsout_code_current,                 \
                                    "get"#Type"(%d)", rs_index);        \
            assert(n_printed >= 0);                                     \
            rsout_code_current += n_printed;                            \
        }                                                               \
        else                                                            \
        {                                                               \
            int n_printed = sprintf(rsout_code_current,                 \
                                    "get"#Type"(%d)", rsid_current);    \
            assert(n_printed >= 0);                                     \
            rsout_code_current += n_printed;                            \
            a = (type*)((byte*)robot_state_first_address+rsid_current); \
            rsid_current += sizeof(type);                               \
        }                                                               \
        return jos;                                                     \
    }

extern "C" void jniMain(JNIEnv *, jobject);

#define java_out_file_prefix "./java/com/qualcomm/ftcrobotcontroller/opmodes/"
#define java_out_file_suffix ".java"

FILE * java_output_file;
uint java_output_name_part_len = 0;
uint java_output_path_part_len = 0;
char * java_output_filename = 0;

int input_path_part_len = 0;
int input_name_part_len = 0;
const char * input_filename = __BASE_FILE__;

int main(int n_args, char ** args)
{
    for(int a = 1; a < n_args; a++) //the first argument is the command
    {
        if(args[a][0] == '-')
        {
            switch(args[a][1])
            {
                case 'o':
                    java_output_filename = args[++a];
            }
        }
    }
    
    for(int l = 0; input_filename[l]; l++)
    {
        if(input_filename[l] == '/') input_path_part_len = l+1;
    }
    
    for(int l = 0; input_filename[l]; l++)
    {
        if(input_filename[l] == '.') input_name_part_len = l;
    }
    
    {
        if(java_output_filename == 0)
        {
            java_output_filename = (char *) malloc(sizeof(java_out_file_prefix)-1
                                                   +input_name_part_len-input_path_part_len
                                                   +sizeof(java_out_file_suffix));
            strncpy(java_output_filename, java_out_file_prefix, sizeof(java_out_file_prefix)-1);
            strncpy(java_output_filename+sizeof(java_out_file_prefix)-1, input_filename+input_path_part_len, input_name_part_len-input_path_part_len);
            strncpy(java_output_filename+sizeof(java_out_file_prefix)-1+input_name_part_len-input_path_part_len, java_out_file_suffix, sizeof(java_out_file_suffix));
        }
        
        java_output_file = fopen(java_output_filename, "w+");
        if(!java_output_file)
        {
            printf("error, could not open file \"%s\"\n", java_output_filename);
            exit(EXIT_SUCCESS);
        }
    }
    
    java_output_path_part_len = 0;
    for(int l = 0; java_output_filename[l]; l++)
    {
        if(java_output_filename[l] == '/') java_output_path_part_len = l+1;
    }
    
    java_output_name_part_len = 0;
    for(int l = 0; java_output_filename[l]; l++)
    {
        if(java_output_filename[l] == '.') java_output_name_part_len = l;
    }
    ////////////////////////////////////////////////////
    
    jniMain(0, 0);
    return 0;
}

//char not included because it has different sizes in c and java
enum types{ type_bool, type_byte, type_int, type_long, type_float, type_double, n_types };
const char * type_java_names[] =
{            "boolean",   "byte",     "int",    "long",    "float",    "double"};
const char * type_names[] =
{               "bool",   "byte",     "int",    "long",    "float",    "double"};
const int type_sizes[] =
{                   1,         1,        4,         8,          4,          8};

//TODO: prettier formatting
void jniGenerate()
{
    fprintf(java_output_file,
            "/*\n"
            "WARNING: this is a generated file\n"
            "changes made this file are not permanent\n"
            "*/\n"
            "package com.qualcomm.ftcrobotcontroller.opmodes;\n"
            ""
            "\n");
    
    fprintf(java_output_file,
            "\n"
            "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n"
            "\n"
            "import com.qualcomm.ftccommon.DbgLog;"
            "\n"
            "import java.nio.ByteBuffer;\n"
            "import java.nio.ByteOrder;\n");
    if(jni_import_string) fprintf(java_output_file, "\n%s\n", jni_import_string);
    
    fprintf(java_output_file,
            "\n"
            "public class %.*s extends LinearOpMode {\n"
            "public static byte[] robot_state;\n"
            "int rsid_current = 0;\n"
            "public %.*s()\n"
            "{\n"
            "    DbgLog.error(\"opmode constructor\");\n"
            /* "    rsid_current = 0;" */
            "    robot_state = new byte[%d];\n",
            java_output_name_part_len-java_output_path_part_len, java_output_filename+java_output_path_part_len,
            java_output_name_part_len-java_output_path_part_len, java_output_filename+java_output_path_part_len,
            rsid_current);
    if(jni_constructor_string) fprintf(java_output_file, "\n%s\n", jni_constructor_string);
    fprintf(java_output_file,
            "}\n"
            "\n");
    
    if(jni_misc_string) fprintf(java_output_file, "\n%s\n", jni_misc_string);
    
    //TODO: make bools work, and bytes
    for(int i = 2; i < n_types; i++)
    {
        const char * type = type_java_names[i];
        #define Type type[0]+'A'-'a', (type+1)
        
        uint buffer_read_size = type_sizes[i];
        
        fprintf(java_output_file,
                "public void set%c%s(int index, %s a)\n"
                "{\n"
                "    rsid_current = index;\n"
                "    ByteBuffer.wrap(robot_state, rsid_current, %d).order(ByteOrder.nativeOrder()).put%c%s(a);\n"
                "    rsid_current = index+%d;\n"
                "}\n"
                "public void setRelative(%s a)\n"
                "{\n"
                "    ByteBuffer.wrap(robot_state, rsid_current, %d).order(ByteOrder.nativeOrder()).put%c%s(a);\n"
                "    rsid_current += %d;\n"
                "}\n"
                "public %s get%c%s(int index)\n"
                "{\n"
                "    rsid_current = index+%d;\n"
                "    return ByteBuffer.wrap(robot_state, index, %d).order(ByteOrder.nativeOrder()).get%c%s();\n"
                "}\n"
                "public %s getRelative%c%s()\n"
                "{\n"
                "    %s out = ByteBuffer.wrap(robot_state, rsid_current, %d).order(ByteOrder.nativeOrder()).get%c%s();\n"
                "    rsid_current += %d;\n"
                "    return out;\n"
                "}\n\n",
                Type, type, buffer_read_size, Type, buffer_read_size,
                type, buffer_read_size, Type, buffer_read_size,
                type, Type, buffer_read_size, buffer_read_size, Type,
                type, Type, type, buffer_read_size, Type, buffer_read_size);
    }
    
    fprintf(java_output_file, "\n"
            "void robotStateOut()\n"
            "{\n"
            "rsid_current = 0;\n"
            "%s\n"
            "}\n", rsout_code);
    
    fprintf(java_output_file, "\n"
            "void robotStateIn()\n"
            "{\n"
            "%s\n"
            "}\n" , rsin_code);
    
    if(jni_variables_string) fprintf(java_output_file, "%s\n", jni_variables_string);
    fprintf(java_output_file, "\n"
            "native void main();\n"
            "\n"
            "static\n"
            "{\n"
            "    System.loadLibrary(\"native_robot\");\n"
            "}\n");
    fprintf(java_output_file,
            "\n"
            "@Override public void runOpMode() throws InterruptedException\n"
            "{\n");
    if(jni_run_opmode_string) fprintf(java_output_file, "%s\n", jni_run_opmode_string);
    fprintf(java_output_file,
            "    main();\n"
            "}\n");
    
    fprintf(java_output_file, "}\n");
    exit(EXIT_SUCCESS);
}
////////////////////////////
#else //compiling normally//
////////////////////////////

#include <jni.h>

jmethodID waitForStartID;
jmethodID waitOneFullHardwareCycleID;
jmethodID waitForNextHardwareCycleID;
jmethodID robotStateInID;
jmethodID robotStateOutID;

#define waitForStart() env->CallVoidMethod(self, waitForStartID)
#define waitOneFullHardwareCycle() env->CallVoidMethod(self, waitOneFullHardwareCycleID)
#define waitForNextHardwareCycle() env->CallVoidMethod(self, waitForNextHardwareCycleID)
#define robotStateIn() env->CallVoidMethod(self, robotStateInID)
#define robotStateOut() env->CallVoidMethod(self, robotStateOutID)

jbyteArray jrobot_state;

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
    jfieldID jrobot_stateID = env->GetStaticFieldID(cls, "robot_state", "[B");
    jrobot_state = (jbyteArray) env->GetStaticObjectField(cls, jrobot_stateID);
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
    
    rsid_current = 0;
}

void cleanupJNI();

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

#define defineJniIn(type, Type)                                 \
    type * jni##Type##In_line(const char * s, const char * filename, int line) \
    {                                                           \
        type * out = (type *)(robot_state.state+rsid_current);  \
        rsid_current += sizeof(type);                           \
        return out;                                             \
    }

void * jniStructIn_line(int struct_size, const char * s, const char * filename, int line)
{
    void * out = robot_state.state+rsid_current;
    rsid_current += struct_size;
    return out;
}

void jniGenerate(){}

jniOutStruct operator, (jniOutStruct jos, const char * s)
{
    return jos;
}

#define defineJniOut(type, Type)                                \
    jniOutStruct operator, (jniOutStruct jos, type * & a)       \
    {                                                           \
        int rs_index = ((byte*)a-(byte*)robot_state.state);     \
        if(rs_index >= 0 && rs_index < rsid_current)            \
        {                                                       \
            /*don't need to do anything*/                       \
        }                                                       \
        else                                                    \
        {                                                       \
            a = (type *) (robot_state.state+rsid_current);      \
            rsid_current += sizeof(type);                       \
        }                                                       \
        return jos;                                             \
    }
#endif

//TODO(if needed): error line numbers and filenames, jniOut currently has no errors
#define jniOut(...) (jniOutVar, __VA_ARGS__, "\n")

#define defineJni(type, Type) defineJniOut(type, Type) defineJniIn(type, Type)

//TODO: byte doesn't work, it needs special cases since it has a different syntax with a byte buffer
//defineJni(byte, Byte)
//not using defineJni because byte has a different syntax for putting into a byte buffer
//#define jniByteIn(s) jniByteIn_line(s, __FILE__, __LINE__)

defineJni(int, Int)
#define jniIntIn(s) jniIntIn_line(s, __FILE__, __LINE__)

defineJni(long, Long)
#define jniLongIn(s) jniLongIn_line(s, __FILE__, __LINE__)

defineJni(float, Float)
#define jniFloatIn(s) jniFloatIn_line(s, __FILE__, __LINE__)

defineJni(double, Double)
#define jniDoubleIn(s) jniDoubleIn_line(s, __FILE__, __LINE__)

#define jniStructIn(type, s) ((type *) jniStructIn_line(sizeof(type), s, __FILE__, __LINE__))

#endif
