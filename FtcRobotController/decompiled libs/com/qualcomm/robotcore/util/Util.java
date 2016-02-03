/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.widget.TextView
 */
package com.qualcomm.robotcore.util;

import android.widget.TextView;
import com.qualcomm.robotcore.util.RobotLog;
import java.io.File;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Random;

public class Util {
    public static String ASCII_RECORD_SEPARATOR = "\u001e";
    public static final String LOWERCASE_ALPHA_NUM_CHARACTERS = "0123456789qwertyuiopasdfghjklzxcvbnm";

    public static String getRandomString(int stringLength, String charSet) {
        Random random = new Random();
        StringBuilder stringBuilder = new StringBuilder();
        for (int i = 0; i < stringLength; ++i) {
            stringBuilder.append(charSet.charAt(random.nextInt(charSet.length())));
        }
        return stringBuilder.toString();
    }

    public static void sortFilesByName(File[] files) {
        Arrays.sort(files, new Comparator<File>(){

            public int a(File file, File file2) {
                return file.getName().compareTo(file2.getName());
            }

            @Override
            public /* synthetic */ int compare(Object object, Object object2) {
                return this.a((File)object, (File)object2);
            }
        });
    }

    public static void updateTextView(final TextView textView, final String msg) {
        if (textView != null) {
            textView.post(new Runnable(){

                @Override
                public void run() {
                    textView.setText((CharSequence)msg);
                }
            });
        }
    }

    public static byte[] concatenateByteArrays(byte[] first, byte[] second) {
        byte[] arrby = new byte[first.length + second.length];
        System.arraycopy(first, 0, arrby, 0, first.length);
        System.arraycopy(second, 0, arrby, first.length, second.length);
        return arrby;
    }

    /*
     * Enabled aggressive block sorting
     * Enabled unnecessary exception pruning
     */
    public static void logThreadLifeCycle(String name, Runnable runnable) {
        try {
            Thread.currentThread().setName(name);
            RobotLog.v(String.format("thread: '%s' starting...", name));
            runnable.run();
        }
        catch (Throwable var2_2) {
            RobotLog.v(String.format("thread: ...terminating '%s'", name));
            throw var2_2;
        }
        RobotLog.v(String.format("thread: ...terminating '%s'", name));
    }

}

