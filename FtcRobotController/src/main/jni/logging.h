#ifndef LOGGING
#define LOGGING

#define logging_jni_import_string "import android.os.Environment;\n"

#define logging_jni_misc_string                                         \
    "public boolean isExternalStorageWritable()\n"                      \
    "{\n"                                                               \
    "    String state = Environment.getExternalStorageState();\n"       \
    "    if (Environment.MEDIA_MOUNTED.equals(state)) {\n"              \
    "        return true;\n"                                            \
    "    }\n"                                                           \
    "    return false;\n"                                               \
    "}\n"

#define MAX_LOGFILES 100

FILE * logfile;

#ifndef GENERATE

#include "jni_functions.h"

#include <android/log.h>
#include <unistd.h>
#include <fcntl.h>

#define initLogfile() initLogfile__file__(__FILE__)
void initLogfile__file__(const char * __file__)
{
    if(!env->CallBooleanMethod(self, env->GetMethodID(cls, "isExternalStorageWritable", "()Z")))
    {
        return;
    }
        
    jclass environmentClass = env->FindClass("android/os/Environment");
    jclass fileClass = env->FindClass("java/io/File");
        
    jstring jpath = (jstring) env->CallObjectMethod(
        env->CallStaticObjectMethod(
            environmentClass,
            env->GetStaticMethodID(environmentClass, "getExternalStoragePublicDirectory", "(Ljava/lang/String;)Ljava/io/File;"),
            env->GetStaticObjectField(
                environmentClass,
                env->GetStaticFieldID(environmentClass, "DIRECTORY_DOWNLOADS", "Ljava/lang/String;"))),
        env->GetMethodID(fileClass, "getAbsolutePath", "()Ljava/lang/String;"));

    logfile = 0;
    if(env->ExceptionOccurred() == 0)
    {
        jsize path_len =  env->GetStringUTFLength(jpath);
        const char * path = env->GetStringUTFChars(jpath, 0);
            
        //NOTE: this does not handle if the total filename is longer than 100 chars
        char * filepath = (char *) malloc(path_len + 100);
            
        for(int log_number = 0; log_number < MAX_LOGFILES; log_number++)
        {
            sprintf(filepath, "%.*s/%.*s_log_%d.txt",
                    path_len, path,
                    (strrchr(__file__, '/') ? ((uint)strrchr(__file__, '.'))-((uint)strrchr(__file__, '/')+1) : sizeof(__file__)),
                    (strrchr(__file__, '/') ? strrchr(__file__, '/') + 1 : __file__),
                    log_number);

            __android_log_print(ANDROID_LOG_DEBUG, "KillerRabbit", "checking filepath %s", filepath);
                
            //int logfile = open(filepath, O_CREAT | O_WRONLY | O_EXCL | O_NONBLOCK,
            //                   S_IRUST | S_IWUSR);
            //NOTE: this is more secure since it does not allow a file to be created between checking and obtaining a filehandle
                
            if(access(filepath, F_OK) == -1)
            {
                logfile = fopen(filepath, "w");
                break;
            }
        }
            
        env->ReleaseStringUTFChars(jpath, path);
    }
    else
    {
        env->ExceptionDescribe();
    }
}

#define android_log(...) __android_log_print(ANDROID_LOG_DEBUG, "KillerRabbit", __VA_ARGS__);

#if 0
#define log(...) android_log(__VA_ARGS__); if(logfile) fprintf(logfile, __VA_ARGS__);
#else
#define log(...) if(logfile) fprintf(logfile, __VA_ARGS__);
#endif

#define closeLogfile() if(logfile) fclose(logfile);

#else

void initLogfile(){}

#define log(...)

#define closeLogfile()

#endif

#endif
