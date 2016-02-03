/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.SerialNumber
 */
package com.qualcomm.hardware.modernrobotics;

import com.qualcomm.hardware.modernrobotics.ReadWriteRunnable;
import com.qualcomm.hardware.modernrobotics.ReadWriteRunnableStandard;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class ReadWriteRunnableBlocking
extends ReadWriteRunnableStandard {
    protected final Lock blockingLock = new ReentrantLock();
    protected final Lock waitingLock = new ReentrantLock();
    protected final Condition blockingCondition = this.blockingLock.newCondition();
    protected final Condition waitingCondition = this.waitingLock.newCondition();
    protected ReadWriteRunnable.BlockingState blockingState = ReadWriteRunnable.BlockingState.BLOCKING;
    private volatile boolean a = false;

    public ReadWriteRunnableBlocking(SerialNumber serialNumber, RobotUsbDevice device, int monitorLength, int startAddress, boolean debug) {
        super(serialNumber, device, monitorLength, startAddress, debug);
    }

    @Override
    public void blockUntilReady() throws RobotCoreException, InterruptedException {
        try {
            this.blockingLock.lock();
            while (this.blockingState == ReadWriteRunnable.BlockingState.BLOCKING) {
                this.blockingCondition.await(100, TimeUnit.MILLISECONDS);
                if (!this.shutdownComplete) continue;
                RobotLog.w((String)("sync device block requested, but device is shut down - " + (Object)this.serialNumber));
                RobotLog.setGlobalErrorMsg((String)"There were problems communicating with a Modern Robotics USB device for an extended period of time.");
                throw new RobotCoreException("cannot block, device is shut down");
            }
        }
        finally {
            this.blockingLock.unlock();
        }
    }

    @Override
    public void startBlockingWork() {
        try {
            this.waitingLock.lock();
            this.blockingState = ReadWriteRunnable.BlockingState.BLOCKING;
            this.waitingCondition.signalAll();
        }
        finally {
            this.waitingLock.unlock();
        }
    }

    @Override
    public void write(int address, byte[] data) {
        byte[] arrby = this.localDeviceWriteCache;
        synchronized (arrby) {
            System.arraycopy(data, 0, this.localDeviceWriteCache, address, data.length);
            this.a = true;
        }
    }

    @Override
    public boolean writeNeeded() {
        return this.a;
    }

    @Override
    public void setWriteNeeded(boolean set) {
        this.a = set;
    }

    @Override
    protected void waitForSyncdEvents() throws RobotCoreException, InterruptedException {
        try {
            this.blockingLock.lock();
            this.blockingState = ReadWriteRunnable.BlockingState.WAITING;
            this.blockingCondition.signalAll();
        }
        finally {
            this.blockingLock.unlock();
        }
        try {
            this.waitingLock.lock();
            while (this.blockingState == ReadWriteRunnable.BlockingState.WAITING) {
                this.waitingCondition.await();
                if (!this.shutdownComplete) continue;
                RobotLog.w((String)("wait for sync'd events requested, but device is shut down - " + (Object)this.serialNumber));
                throw new RobotCoreException("cannot block, device is shut down");
            }
        }
        finally {
            this.waitingLock.unlock();
        }
    }
}

