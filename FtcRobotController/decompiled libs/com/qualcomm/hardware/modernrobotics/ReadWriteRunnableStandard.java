/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.modernrobotics.ReadWriteRunnableUsbHandler
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.SerialNumber
 *  com.qualcomm.robotcore.util.TypeConversion
 *  com.qualcomm.robotcore.util.Util
 */
package com.qualcomm.hardware.modernrobotics;

import com.qualcomm.hardware.modernrobotics.ReadWriteRunnable;
import com.qualcomm.hardware.modernrobotics.ReadWriteRunnableSegment;
import com.qualcomm.modernrobotics.ReadWriteRunnableUsbHandler;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.TypeConversion;
import com.qualcomm.robotcore.util.Util;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CountDownLatch;

public class ReadWriteRunnableStandard
implements ReadWriteRunnable {
    protected final byte[] localDeviceReadCache = new byte[256];
    protected final byte[] localDeviceWriteCache = new byte[256];
    protected Map<Integer, ReadWriteRunnableSegment> segments = new HashMap<Integer, ReadWriteRunnableSegment>();
    protected ConcurrentLinkedQueue<Integer> segmentReadQueue = new ConcurrentLinkedQueue();
    protected ConcurrentLinkedQueue<Integer> segmentWriteQueue = new ConcurrentLinkedQueue();
    protected final SerialNumber serialNumber;
    protected ReadWriteRunnableUsbHandler usbHandler;
    protected int startAddress;
    protected int monitorLength;
    protected volatile CountDownLatch runningInterlock = new CountDownLatch(1);
    protected volatile boolean running = false;
    protected volatile boolean shutdownComplete = false;
    private volatile boolean a = false;
    protected ReadWriteRunnable.Callback callback;
    protected final boolean DEBUG_LOGGING;

    public ReadWriteRunnableStandard(SerialNumber serialNumber, RobotUsbDevice device, int monitorLength, int startAddress, boolean debug) {
        this.serialNumber = serialNumber;
        this.startAddress = startAddress;
        this.monitorLength = monitorLength;
        this.DEBUG_LOGGING = debug;
        this.callback = new ReadWriteRunnable.EmptyCallback();
        this.usbHandler = new ReadWriteRunnableUsbHandler(device);
    }

    @Override
    public void setCallback(ReadWriteRunnable.Callback callback) {
        this.callback = callback;
    }

    @Override
    public void blockUntilReady() throws RobotCoreException, InterruptedException {
        if (this.shutdownComplete) {
            RobotLog.w((String)("sync device block requested, but device is shut down - " + (Object)this.serialNumber));
            RobotLog.setGlobalErrorMsg((String)"There were problems communicating with a Modern Robotics USB device for an extended period of time.");
            throw new RobotCoreException("cannot block, device is shut down");
        }
    }

    @Override
    public void startBlockingWork() {
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
    public void write(int address, byte[] data) {
        byte[] arrby = this.localDeviceWriteCache;
        synchronized (arrby) {
            System.arraycopy(data, 0, this.localDeviceWriteCache, address, data.length);
            this.a = true;
        }
    }

    @Override
    public byte[] readFromWriteCache(int address, int size) {
        byte[] arrby = this.localDeviceWriteCache;
        synchronized (arrby) {
            return Arrays.copyOfRange(this.localDeviceWriteCache, address, address + size);
        }
    }

    @Override
    public byte[] read(int address, int size) {
        byte[] arrby = this.localDeviceReadCache;
        synchronized (arrby) {
            return Arrays.copyOfRange(this.localDeviceReadCache, address, address + size);
        }
    }

    @Override
    public void close() {
        this.running = false;
        try {
            this.blockUntilReady();
            this.startBlockingWork();
        }
        catch (InterruptedException var1_1) {
            RobotLog.w((String)("Exception while closing USB device: " + var1_1.getMessage()));
        }
        catch (RobotCoreException var1_2) {
            RobotLog.w((String)("Exception while closing USB device: " + var1_2.getMessage()));
        }
        while (!this.shutdownComplete) {
            Thread.yield();
        }
    }

    @Override
    public ReadWriteRunnableSegment createSegment(int key, int address, int size) {
        ReadWriteRunnableSegment readWriteRunnableSegment = new ReadWriteRunnableSegment(address, size);
        this.segments.put(key, readWriteRunnableSegment);
        return readWriteRunnableSegment;
    }

    @Override
    public void destroySegment(int key) {
        this.segments.remove(key);
    }

    @Override
    public ReadWriteRunnableSegment getSegment(int key) {
        return this.segments.get(key);
    }

    @Override
    public void queueSegmentRead(int key) {
        this.queueIfNotAlreadyQueued(key, this.segmentReadQueue);
    }

    @Override
    public void queueSegmentWrite(int key) {
        this.queueIfNotAlreadyQueued(key, this.segmentWriteQueue);
    }

    @Override
    public void awaitRunning() throws InterruptedException {
        this.runningInterlock.await();
    }

    @Override
    public void run() {
        Util.logThreadLifeCycle((String)String.format("r/w loop for device %s", new Object[]{this.serialNumber}), (Runnable)new Runnable(){

            /*
             * Exception decompiling
             */
            @Override
            public void run() {
                // This method has failed to decompile.  When submitting a bug report, please provide this stack trace, and (if you hold appropriate legal rights) the relevant class file.
                // java.lang.NullPointerException
                // org.benf.cfr.reader.bytecode.analysis.opgraph.Op03Blocks$Block3.getLastUnconditionalBackjumpToHere(Op03Blocks.java:1107)
                // org.benf.cfr.reader.bytecode.analysis.opgraph.Op03Blocks.detectMoves(Op03Blocks.java:404)
                // org.benf.cfr.reader.bytecode.analysis.opgraph.Op03Blocks.topologicalSort(Op03Blocks.java:877)
                // org.benf.cfr.reader.bytecode.CodeAnalyser.getAnalysisInner(CodeAnalyser.java:512)
                // org.benf.cfr.reader.bytecode.CodeAnalyser.getAnalysisOrWrapFail(CodeAnalyser.java:213)
                // org.benf.cfr.reader.bytecode.CodeAnalyser.getAnalysis(CodeAnalyser.java:158)
                // org.benf.cfr.reader.entities.attributes.AttributeCode.analyse(AttributeCode.java:91)
                // org.benf.cfr.reader.entities.Method.analyse(Method.java:353)
                // org.benf.cfr.reader.entities.ClassFile.analyseMid(ClassFile.java:731)
                // org.benf.cfr.reader.entities.ClassFile.analyseInnerClassesPass1(ClassFile.java:644)
                // org.benf.cfr.reader.entities.ClassFile.analyseMid(ClassFile.java:727)
                // org.benf.cfr.reader.entities.ClassFile.analyseTop(ClassFile.java:663)
                // org.benf.cfr.reader.Main.doJar(Main.java:126)
                // org.benf.cfr.reader.Main.main(Main.java:178)
                throw new IllegalStateException("Decompilation failed");
            }
        });
    }

    protected void waitForSyncdEvents() throws RobotCoreException, InterruptedException {
    }

    protected void dumpBuffers(String name, byte[] byteArray) {
        RobotLog.v((String)("Dumping " + name + " buffers for " + (Object)this.serialNumber));
        StringBuilder stringBuilder = new StringBuilder(1024);
        for (int i = 0; i < this.startAddress + this.monitorLength; ++i) {
            stringBuilder.append(String.format(" %02x", TypeConversion.unsignedByteToInt((byte)byteArray[i])));
            if ((i + 1) % 16 != 0) continue;
            stringBuilder.append("\n");
        }
        RobotLog.v((String)stringBuilder.toString());
    }

    protected void queueIfNotAlreadyQueued(int key, ConcurrentLinkedQueue<Integer> queue) {
        if (!queue.contains(key)) {
            queue.add(key);
        }
    }

}

