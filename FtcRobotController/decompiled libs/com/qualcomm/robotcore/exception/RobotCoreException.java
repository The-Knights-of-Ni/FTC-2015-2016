/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.exception;

public class RobotCoreException
extends Exception {
    private Exception a = null;

    public RobotCoreException(String message) {
        super(message);
    }

    public /* varargs */ RobotCoreException(String format, Object ... args) {
        super(String.format(format, args));
    }

    public static /* varargs */ RobotCoreException createChained(Exception e, String format, Object ... args) {
        RobotCoreException robotCoreException = new RobotCoreException(format, args);
        robotCoreException.a = e;
        return robotCoreException;
    }

    public boolean isChainedException() {
        return this.a != null;
    }

    public Exception getChainedException() {
        return this.a;
    }
}

