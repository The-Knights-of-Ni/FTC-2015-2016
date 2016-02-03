/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.content.Context
 *  android.os.Environment
 *  android.util.Log
 */
package com.qualcomm.robotcore.util;

import android.content.Context;
import android.os.Environment;
import android.util.Log;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.RunShellCommand;
import java.io.File;

public class RobotLog {
    private static String a = "";
    public static final String TAG = "RobotCore";
    private static boolean b = false;

    private RobotLog() {
    }

    public static /* varargs */ void v(String format, Object ... args) {
        RobotLog.v(String.format(format, args));
    }

    public static void v(String message) {
        Log.v((String)"RobotCore", (String)message);
    }

    public static /* varargs */ void d(String format, Object ... args) {
        RobotLog.d(String.format(format, args));
    }

    public static void d(String message) {
        Log.d((String)"RobotCore", (String)message);
    }

    public static /* varargs */ void i(String format, Object ... args) {
        RobotLog.i(String.format(format, args));
    }

    public static void i(String message) {
        Log.i((String)"RobotCore", (String)message);
    }

    public static /* varargs */ void w(String format, Object ... args) {
        RobotLog.w(String.format(format, args));
    }

    public static void w(String message) {
        Log.w((String)"RobotCore", (String)message);
    }

    public static /* varargs */ void e(String format, Object ... args) {
        RobotLog.e(String.format(format, args));
    }

    public static void e(String message) {
        Log.e((String)"RobotCore", (String)message);
    }

    public static void logStacktrace(Exception e) {
        RobotLog.e(e.toString());
        for (StackTraceElement stackTraceElement : e.getStackTrace()) {
            RobotLog.e(stackTraceElement.toString());
        }
    }

    public static void logStacktrace(RobotCoreException e) {
        RobotLog.e(e.toString());
        for (StackTraceElement stackTraceElement : e.getStackTrace()) {
            RobotLog.e(stackTraceElement.toString());
        }
        if (e.isChainedException()) {
            RobotLog.e("Exception chained from:");
            if (e.getChainedException() instanceof RobotCoreException) {
                RobotLog.logStacktrace((RobotCoreException)e.getChainedException());
            } else {
                RobotLog.logStacktrace(e.getChainedException());
            }
        }
    }

    public static void setGlobalErrorMsg(String message) {
        if (a.isEmpty()) {
            a = a + message;
        }
    }

    public static void setGlobalErrorMsgAndThrow(String message, RobotCoreException e) throws RobotCoreException {
        RobotLog.setGlobalErrorMsg(message + "\n" + e.getMessage());
        throw e;
    }

    public static String getGlobalErrorMsg() {
        return a;
    }

    public static boolean hasGlobalErrorMsg() {
        return !a.isEmpty();
    }

    public static void clearGlobalErrorMsg() {
        a = "";
    }

    public static void logAndThrow(String errMsg) throws RobotCoreException {
        RobotLog.w(errMsg);
        throw new RobotCoreException(errMsg);
    }

    public static void writeLogcatToDisk(Context context, final int fileSizeKb) {
        if (b) {
            return;
        }
        b = true;
        final String string = context.getPackageName();
        final String string2 = new File(RobotLog.getLogFilename(context)).getAbsolutePath();
        Thread thread = new Thread("Logging Thread"){

            @Override
            public void run() {
                try {
                    String string3 = "UsbRequestJNI:S UsbRequest:S *:V";
                    boolean bl = true;
                    RobotLog.v("saving logcat to " + string2);
                    RunShellCommand runShellCommand = new RunShellCommand();
                    RunShellCommand.killSpawnedProcess("logcat", string, runShellCommand);
                    runShellCommand.run(String.format("logcat -f %s -r%d -n%d -v time %s", string2, fileSizeKb, 1, "UsbRequestJNI:S UsbRequest:S *:V"));
                }
                catch (RobotCoreException var1_2) {
                    RobotLog.v("Error while writing log file to disk: " + var1_2.toString());
                }
                finally {
                    b = false;
                }
            }
        };
        thread.start();
    }

    public static String getLogFilename(Context context) {
        String string = Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + context.getPackageName();
        return string + ".logcat";
    }

    public static void cancelWriteLogcatToDisk(Context context) {
        final String string = context.getPackageName();
        final String string2 = new File(Environment.getExternalStorageDirectory(), string).getAbsolutePath();
        b = false;
        Thread thread = new Thread(){

            @Override
            public void run() {
                try {
                    Thread.sleep(1000);
                }
                catch (InterruptedException var1_1) {
                    // empty catch block
                }
                try {
                    RobotLog.v("closing logcat file " + string2);
                    RunShellCommand runShellCommand = new RunShellCommand();
                    RunShellCommand.killSpawnedProcess("logcat", string, runShellCommand);
                }
                catch (RobotCoreException var1_3) {
                    RobotLog.v("Unable to cancel writing log file to disk: " + var1_3.toString());
                }
            }
        };
        thread.start();
    }

}

