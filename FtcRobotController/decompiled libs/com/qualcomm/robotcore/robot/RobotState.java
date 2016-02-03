/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.robot;

import com.qualcomm.robotcore.util.RobotLog;

public enum RobotState {
    NOT_STARTED(0),
    INIT(1),
    RUNNING(2),
    STOPPED(3),
    EMERGENCY_STOP(4),
    DROPPED_CONNECTION(5);
    
    private int a;
    private static final RobotState[] b;

    private RobotState(int state) {
        this.a = state;
    }

    public byte asByte() {
        return (byte)this.a;
    }

    public static RobotState fromByte(byte b) {
        RobotState robotState = NOT_STARTED;
        try {
            robotState = RobotState.b[b];
        }
        catch (ArrayIndexOutOfBoundsException var2_2) {
            RobotLog.w(String.format("Cannot convert %d to RobotState: %s", Byte.valueOf(b), var2_2.toString()));
        }
        return robotState;
    }

    static {
        b = RobotState.values();
    }
}

