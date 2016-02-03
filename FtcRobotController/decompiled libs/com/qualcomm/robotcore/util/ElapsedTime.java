/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.util;

import com.qualcomm.robotcore.util.RobotLog;

public class ElapsedTime {
    public static final double dSECOND_IN_NANO = 1.0E9;
    public static final long lSECOND_IN_NANO = 1000000000;
    public static final double dMILLIS_IN_NANO = 1000000.0;
    public static final long lMILLIS_IN_NANO = 1000000;
    private long a = 0;
    private double b = 1.0E9;

    public ElapsedTime() {
        this.reset();
    }

    public ElapsedTime(long startTime) {
        this.a = startTime;
    }

    public ElapsedTime(Resolution resolution) {
        this.reset();
        switch (resolution) {
            case SECONDS: {
                this.b = 1.0E9;
                break;
            }
            case MILLISECONDS: {
                this.b = 1000000.0;
            }
        }
    }

    public void reset() {
        this.a = System.nanoTime();
    }

    public double startTime() {
        return (double)this.a / this.b;
    }

    public double time() {
        return (double)(System.nanoTime() - this.a) / this.b;
    }

    private String a() {
        if (this.b == 1.0E9) {
            return "seconds";
        }
        if (this.b == 1000000.0) {
            return "milliseconds";
        }
        return "Unknown units";
    }

    public void log(String label) {
        RobotLog.v(String.format("TIMER: %20s - %1.3f %s", label, this.time(), this.a()));
    }

    public String toString() {
        return String.format("%1.4f %s", this.time(), this.a());
    }

    public static enum Resolution {
        SECONDS,
        MILLISECONDS;
        

        private Resolution() {
        }
    }

}

