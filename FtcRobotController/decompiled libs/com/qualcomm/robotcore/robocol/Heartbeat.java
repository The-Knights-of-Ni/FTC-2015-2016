/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.robocol;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robocol.RobocolParsable;
import com.qualcomm.robotcore.robocol.RobocolParsableBase;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.RobotLog;
import java.nio.BufferOverflowException;
import java.nio.ByteBuffer;

public class Heartbeat
extends RobocolParsableBase {
    public static final short PAYLOAD_SIZE = 11;
    public static final short BUFFER_SIZE = 16;
    private long a;
    private RobotState b;

    public Heartbeat() {
        this.a = System.nanoTime();
        this.b = RobotState.NOT_STARTED;
    }

    public Heartbeat(Token token) {
        switch (token) {
            case EMPTY: {
                this.a = 0;
                this.b = RobotState.NOT_STARTED;
            }
        }
    }

    public long getTimestamp() {
        return this.a;
    }

    public double getElapsedTime() {
        return (double)(System.nanoTime() - this.a) / 1.0E9;
    }

    @Override
    public RobocolParsable.MsgType getRobocolMsgType() {
        return RobocolParsable.MsgType.HEARTBEAT;
    }

    public byte getRobotState() {
        return this.b.asByte();
    }

    public void setRobotState(RobotState state) {
        this.b = state;
    }

    @Override
    public byte[] toByteArray() throws RobotCoreException {
        ByteBuffer byteBuffer = this.getWriteBuffer(11);
        try {
            byteBuffer.putLong(this.a);
            byteBuffer.put(this.b.asByte());
        }
        catch (BufferOverflowException var2_2) {
            RobotLog.logStacktrace(var2_2);
        }
        return byteBuffer.array();
    }

    @Override
    public void fromByteArray(byte[] byteArray) throws RobotCoreException {
        if (byteArray.length < 16) {
            throw new RobotCoreException("Expected buffer of at least 16 bytes, received " + byteArray.length);
        }
        ByteBuffer byteBuffer = this.getReadBuffer(byteArray);
        this.a = byteBuffer.getLong();
        this.b = RobotState.fromByte(byteBuffer.get());
    }

    public String toString() {
        return String.format("Heartbeat - seq: %4d, time: %d", this.getSequenceNumber(), this.a);
    }

    public static enum Token {
        EMPTY;
        

        private Token() {
        }
    }

}

