/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.eventloop.SyncdDevice
 *  com.qualcomm.robotcore.exception.RobotCoreException
 */
package com.qualcomm.hardware.modernrobotics;

import com.qualcomm.hardware.modernrobotics.ReadWriteRunnableSegment;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.exception.RobotCoreException;

public interface ReadWriteRunnable
extends SyncdDevice,
Runnable {
    public static final int MAX_BUFFER_SIZE = 256;

    public void setCallback(Callback var1);

    public void blockUntilReady() throws RobotCoreException, InterruptedException;

    public void startBlockingWork();

    public boolean writeNeeded();

    public void setWriteNeeded(boolean var1);

    public void write(int var1, byte[] var2);

    public byte[] readFromWriteCache(int var1, int var2);

    public byte[] read(int var1, int var2);

    public void close();

    public ReadWriteRunnableSegment createSegment(int var1, int var2, int var3);

    public void destroySegment(int var1);

    public ReadWriteRunnableSegment getSegment(int var1);

    public void queueSegmentRead(int var1);

    public void queueSegmentWrite(int var1);

    @Override
    public void run();

    public void awaitRunning() throws InterruptedException;

    public static class EmptyCallback
    implements Callback {
        @Override
        public void readComplete() throws InterruptedException {
        }

        @Override
        public void writeComplete() throws InterruptedException {
        }
    }

    public static interface Callback {
        public void readComplete() throws InterruptedException;

        public void writeComplete() throws InterruptedException;
    }

    public static enum BlockingState {
        BLOCKING,
        WAITING;
        

        private BlockingState() {
        }
    }

}

