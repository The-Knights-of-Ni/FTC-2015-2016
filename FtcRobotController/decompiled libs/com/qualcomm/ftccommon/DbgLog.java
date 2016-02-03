/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.util.Log
 */
package com.qualcomm.ftccommon;

import android.util.Log;

public class DbgLog {
    public static final String TAG = "FIRST";
    public static final String ERROR_PREPEND = "### ERROR: ";

    private DbgLog() {
    }

    public static void msg(String message) {
        Log.i((String)"FIRST", (String)message);
    }

    public static /* varargs */ void msg(String format, Object ... args) {
        DbgLog.msg(String.format(format, args));
    }

    public static void error(String message) {
        Log.e((String)"FIRST", (String)("### ERROR: " + message));
    }

    public static /* varargs */ void error(String format, Object ... args) {
        DbgLog.error(String.format(format, args));
    }

    public static void logStacktrace(Exception e) {
        DbgLog.msg(e.toString());
        for (StackTraceElement stackTraceElement : e.getStackTrace()) {
            DbgLog.msg(stackTraceElement.toString());
        }
    }
}

