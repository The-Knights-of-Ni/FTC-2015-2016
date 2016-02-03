/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.eventloop.EventLoopManager
 *  com.qualcomm.robotcore.eventloop.SyncdDevice
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.SerialNumber
 */
package com.qualcomm.hardware.modernrobotics;

import com.qualcomm.hardware.modernrobotics.ReadWriteRunnable;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public abstract class ModernRoboticsUsbDevice
implements ReadWriteRunnable.Callback {
    public static final int MFG_CODE_MODERN_ROBOTICS = 77;
    public static final int DEVICE_ID_DC_MOTOR_CONTROLLER = 77;
    public static final int DEVICE_ID_SERVO_CONTROLLER = 83;
    public static final int DEVICE_ID_LEGACY_MODULE = 73;
    public static final int DEVICE_ID_DEVICE_INTERFACE_MODULE = 65;
    protected SerialNumber serialNumber;
    protected ExecutorService readWriteService = Executors.newSingleThreadExecutor();
    protected ReadWriteRunnable readWriteRunnable;

    public ModernRoboticsUsbDevice(SerialNumber serialNumber, EventLoopManager manager, ReadWriteRunnable readWriteRunnable) throws RobotCoreException, InterruptedException {
        this.serialNumber = serialNumber;
        this.readWriteRunnable = readWriteRunnable;
        if (readWriteRunnable != null) {
            RobotLog.v((String)("Starting up device " + serialNumber.toString()));
            this.readWriteService.execute(readWriteRunnable);
            readWriteRunnable.blockUntilReady();
            readWriteRunnable.setCallback(this);
            readWriteRunnable.awaitRunning();
            manager.registerSyncdDevice((SyncdDevice)readWriteRunnable);
        }
    }

    public abstract String getDeviceName();

    public SerialNumber getSerialNumber() {
        return this.serialNumber;
    }

    public int getVersion() {
        return this.read(0);
    }

    public void close() {
        RobotLog.v((String)("Shutting down device " + this.serialNumber.toString()));
        this.readWriteService.shutdown();
        this.readWriteRunnable.close();
    }

    public void write(int address, byte data) {
        this.write(address, new byte[]{data});
    }

    public void write(int address, int data) {
        this.write(address, new byte[]{(byte)data});
    }

    public void write(int address, double data) {
        this.write(address, new byte[]{(byte)data});
    }

    public void write(int address, byte[] data) {
        this.readWriteRunnable.write(address, data);
    }

    public byte readFromWriteCache(int address) {
        return this.readFromWriteCache(address, 1)[0];
    }

    public byte[] readFromWriteCache(int address, int size) {
        return this.readWriteRunnable.readFromWriteCache(address, size);
    }

    public byte read(int address) {
        return this.read(address, 1)[0];
    }

    public byte[] read(int address, int size) {
        return this.readWriteRunnable.read(address, size);
    }

    @Override
    public void readComplete() throws InterruptedException {
    }

    @Override
    public void writeComplete() throws InterruptedException {
    }
}

