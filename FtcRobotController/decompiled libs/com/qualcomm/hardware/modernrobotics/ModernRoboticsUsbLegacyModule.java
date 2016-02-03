/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.eventloop.EventLoopManager
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.I2cController
 *  com.qualcomm.robotcore.hardware.I2cController$I2cPortReadyCallback
 *  com.qualcomm.robotcore.hardware.LegacyModule
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
 *  com.qualcomm.robotcore.util.SerialNumber
 */
package com.qualcomm.hardware.modernrobotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDevice;
import com.qualcomm.hardware.modernrobotics.ReadWriteRunnable;
import com.qualcomm.hardware.modernrobotics.ReadWriteRunnableSegment;
import com.qualcomm.hardware.modernrobotics.ReadWriteRunnableStandard;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.SerialNumber;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;

public class ModernRoboticsUsbLegacyModule
extends ModernRoboticsUsbDevice
implements LegacyModule {
    public static final boolean DEBUG_LOGGING = false;
    public static final int MONITOR_LENGTH = 13;
    public static final byte START_ADDRESS = 3;
    public static final byte MIN_PORT_NUMBER = 0;
    public static final byte MAX_PORT_NUMBER = 5;
    public static final byte NUMBER_OF_PORTS = 6;
    public static final byte I2C_ACTION_FLAG = -1;
    public static final byte I2C_NO_ACTION_FLAG = 0;
    public static final byte SIZE_ANALOG_BUFFER = 2;
    public static final byte SIZE_I2C_BUFFER = 27;
    public static final byte SIZE_OF_PORT_BUFFER = 32;
    public static final byte NXT_MODE_ANALOG = 0;
    public static final byte NXT_MODE_I2C = 1;
    public static final byte NXT_MODE_9V_ENABLED = 2;
    public static final byte NXT_MODE_DIGITAL_0 = 4;
    public static final byte NXT_MODE_DIGITAL_1 = 8;
    public static final byte NXT_MODE_READ = -128;
    public static final byte NXT_MODE_WRITE = 0;
    public static final byte BUFFER_FLAG_S0 = 1;
    public static final byte BUFFER_FLAG_S1 = 2;
    public static final byte BUFFER_FLAG_S2 = 4;
    public static final byte BUFFER_FLAG_S3 = 8;
    public static final byte BUFFER_FLAG_S4 = 16;
    public static final byte BUFFER_FLAG_S5 = 32;
    public static final int ADDRESS_BUFFER_STATUS = 3;
    public static final int ADDRESS_ANALOG_PORT_S0 = 4;
    public static final int ADDRESS_ANALOG_PORT_S1 = 6;
    public static final int ADDRESS_ANALOG_PORT_S2 = 8;
    public static final int ADDRESS_ANALOG_PORT_S3 = 10;
    public static final int ADDRESS_ANALOG_PORT_S4 = 12;
    public static final int ADDRESS_ANALOG_PORT_S5 = 14;
    public static final int ADDRESS_I2C_PORT_SO = 16;
    public static final int ADDRESS_I2C_PORT_S1 = 48;
    public static final int ADDRESS_I2C_PORT_S2 = 80;
    public static final int ADDRESS_I2C_PORT_S3 = 112;
    public static final int ADDRESS_I2C_PORT_S4 = 144;
    public static final int ADDRESS_I2C_PORT_S5 = 176;
    public static final byte OFFSET_I2C_PORT_MODE = 0;
    public static final byte OFFSET_I2C_PORT_I2C_ADDRESS = 1;
    public static final byte OFFSET_I2C_PORT_MEMORY_ADDRESS = 2;
    public static final byte OFFSET_I2C_PORT_MEMORY_LENGTH = 3;
    public static final byte OFFSET_I2C_PORT_MEMORY_BUFFER = 4;
    public static final byte OFFSET_I2C_PORT_FLAG = 31;
    public static final int[] ADDRESS_ANALOG_PORT_MAP = new int[]{4, 6, 8, 10, 12, 14};
    public static final int[] ADDRESS_I2C_PORT_MAP = new int[]{16, 48, 80, 112, 144, 176};
    public static final int[] BUFFER_FLAG_MAP = new int[]{1, 2, 4, 8, 16, 32};
    public static final int[] DIGITAL_LINE = new int[]{4, 8};
    public static final int[] PORT_9V_CAPABLE = new int[]{4, 5};
    private final ReadWriteRunnableSegment[] a = new ReadWriteRunnableSegment[12];
    private final I2cController.I2cPortReadyCallback[] b = new I2cController.I2cPortReadyCallback[6];

    public ModernRoboticsUsbLegacyModule(SerialNumber serialNumber, RobotUsbDevice device, EventLoopManager manager) throws RobotCoreException, InterruptedException {
        super(serialNumber, manager, new ReadWriteRunnableStandard(serialNumber, device, 13, 3, false));
        this.readWriteRunnable.setCallback(this);
        for (int i = 0; i < 6; ++i) {
            this.a[i] = this.readWriteRunnable.createSegment(i, ADDRESS_I2C_PORT_MAP[i], 32);
            this.a[i + 6] = this.readWriteRunnable.createSegment(i + 6, ADDRESS_I2C_PORT_MAP[i] + 31, 1);
            this.enableAnalogReadMode(i);
            this.readWriteRunnable.queueSegmentWrite(i);
        }
    }

    @Override
    public String getDeviceName() {
        return "Modern Robotics USB Legacy Module";
    }

    public String getConnectionInfo() {
        return "USB " + (Object)this.getSerialNumber();
    }

    @Override
    public void close() {
        super.close();
    }

    public void registerForI2cPortReadyCallback(I2cController.I2cPortReadyCallback callback, int port) {
        this.b[port] = callback;
    }

    public void deregisterForPortReadyCallback(int port) {
        this.b[port] = null;
    }

    public void enableI2cReadMode(int physicalPort, int i2cAddress, int memAddress, int length) {
        this.a(physicalPort);
        this.b(length);
        try {
            this.a[physicalPort].getWriteLock().lock();
            byte[] arrby = this.a[physicalPort].getWriteBuffer();
            arrby[0] = -127;
            arrby[1] = (byte)i2cAddress;
            arrby[2] = (byte)memAddress;
            arrby[3] = (byte)length;
        }
        finally {
            this.a[physicalPort].getWriteLock().unlock();
        }
    }

    public void enableI2cWriteMode(int physicalPort, int i2cAddress, int memAddress, int length) {
        this.a(physicalPort);
        this.b(length);
        try {
            this.a[physicalPort].getWriteLock().lock();
            byte[] arrby = this.a[physicalPort].getWriteBuffer();
            arrby[0] = 1;
            arrby[1] = (byte)i2cAddress;
            arrby[2] = (byte)memAddress;
            arrby[3] = (byte)length;
        }
        finally {
            this.a[physicalPort].getWriteLock().unlock();
        }
    }

    public void enableAnalogReadMode(int physicalPort) {
        this.a(physicalPort);
        try {
            this.a[physicalPort].getWriteLock().lock();
            byte[] arrby = this.a[physicalPort].getWriteBuffer();
            arrby[0] = 0;
        }
        finally {
            this.a[physicalPort].getWriteLock().unlock();
        }
        this.writeI2cCacheToController(physicalPort);
    }

    public void enable9v(int physicalPort, boolean enable) {
        if (Arrays.binarySearch(PORT_9V_CAPABLE, physicalPort) < 0) {
            throw new IllegalArgumentException("9v is only available on the following ports: " + Arrays.toString(PORT_9V_CAPABLE));
        }
        try {
            this.a[physicalPort].getWriteLock().lock();
            byte by = this.a[physicalPort].getWriteBuffer()[0];
            by = enable ? (byte)(by | 2) : (byte)(by & -3);
            this.a[physicalPort].getWriteBuffer()[0] = by;
        }
        finally {
            this.a[physicalPort].getWriteLock().unlock();
        }
        this.writeI2cCacheToController(physicalPort);
    }

    public void setReadMode(int physicalPort, int i2cAddr, int memAddr, int memLen) {
        this.a(physicalPort);
        try {
            this.a[physicalPort].getWriteLock().lock();
            byte[] arrby = this.a[physicalPort].getWriteBuffer();
            arrby[0] = -127;
            arrby[1] = (byte)i2cAddr;
            arrby[2] = (byte)memAddr;
            arrby[3] = (byte)memLen;
        }
        finally {
            this.a[physicalPort].getWriteLock().unlock();
        }
    }

    public void setWriteMode(int physicalPort, int i2cAddress, int memAddress) {
        this.a(physicalPort);
        try {
            this.a[physicalPort].getWriteLock().lock();
            byte[] arrby = this.a[physicalPort].getWriteBuffer();
            arrby[0] = 1;
            arrby[1] = (byte)i2cAddress;
            arrby[2] = (byte)memAddress;
        }
        finally {
            this.a[physicalPort].getWriteLock().unlock();
        }
    }

    public void setData(int physicalPort, byte[] data, int length) {
        this.a(physicalPort);
        this.b(length);
        try {
            this.a[physicalPort].getWriteLock().lock();
            byte[] arrby = this.a[physicalPort].getWriteBuffer();
            System.arraycopy(data, 0, arrby, 4, length);
            arrby[3] = (byte)length;
        }
        finally {
            this.a[physicalPort].getWriteLock().unlock();
        }
    }

    public void setDigitalLine(int physicalPort, int line, boolean set) {
        this.a(physicalPort);
        this.c(line);
        try {
            this.a[physicalPort].getWriteLock().lock();
            byte by = this.a[physicalPort].getWriteBuffer()[0];
            by = set ? (byte)(by | DIGITAL_LINE[line]) : (byte)(by & ~ DIGITAL_LINE[line]);
            this.a[physicalPort].getWriteBuffer()[0] = by;
        }
        finally {
            this.a[physicalPort].getWriteLock().unlock();
        }
        this.writeI2cCacheToController(physicalPort);
    }

    public byte[] readAnalog(int physicalPort) {
        this.a(physicalPort);
        return this.read(ADDRESS_ANALOG_PORT_MAP[physicalPort], 2);
    }

    public byte[] getCopyOfReadBuffer(int physicalPort) {
        this.a(physicalPort);
        Object object = null;
        try {
            this.a[physicalPort].getReadLock().lock();
            byte[] arrby = this.a[physicalPort].getReadBuffer();
            byte by = arrby[3];
            object = new byte[by];
            System.arraycopy(arrby, 4, object, 0, object.length);
        }
        finally {
            this.a[physicalPort].getReadLock().unlock();
        }
        return object;
    }

    public byte[] getCopyOfWriteBuffer(int physicalPort) {
        this.a(physicalPort);
        Object object = null;
        try {
            this.a[physicalPort].getWriteLock().lock();
            byte[] arrby = this.a[physicalPort].getWriteBuffer();
            byte by = arrby[3];
            object = new byte[by];
            System.arraycopy(arrby, 4, object, 0, object.length);
        }
        finally {
            this.a[physicalPort].getWriteLock().unlock();
        }
        return object;
    }

    public void copyBufferIntoWriteBuffer(int physicalPort, byte[] buffer) {
        this.a(physicalPort);
        this.b(buffer.length);
        try {
            this.a[physicalPort].getWriteLock().lock();
            byte[] arrby = this.a[physicalPort].getWriteBuffer();
            System.arraycopy(buffer, 0, arrby, 4, buffer.length);
        }
        finally {
            this.a[physicalPort].getWriteLock().unlock();
        }
    }

    public void setI2cPortActionFlag(int physicalPort) {
        this.a(physicalPort);
        try {
            this.a[physicalPort].getWriteLock().lock();
            byte[] arrby = this.a[physicalPort].getWriteBuffer();
            arrby[31] = -1;
        }
        finally {
            this.a[physicalPort].getWriteLock().unlock();
        }
    }

    public boolean isI2cPortActionFlagSet(int physicalPort) {
        this.a(physicalPort);
        boolean bl = false;
        try {
            this.a[physicalPort].getReadLock().lock();
            byte[] arrby = this.a[physicalPort].getReadBuffer();
            bl = arrby[31] == -1;
        }
        finally {
            this.a[physicalPort].getReadLock().unlock();
        }
        return bl;
    }

    public void readI2cCacheFromController(int physicalPort) {
        this.a(physicalPort);
        this.readWriteRunnable.queueSegmentRead(physicalPort);
    }

    public void writeI2cCacheToController(int physicalPort) {
        this.a(physicalPort);
        this.readWriteRunnable.queueSegmentWrite(physicalPort);
    }

    public void writeI2cPortFlagOnlyToController(int physicalPort) {
        this.a(physicalPort);
        ReadWriteRunnableSegment readWriteRunnableSegment = this.a[physicalPort];
        ReadWriteRunnableSegment readWriteRunnableSegment2 = this.a[physicalPort + 6];
        try {
            readWriteRunnableSegment.getWriteLock().lock();
            readWriteRunnableSegment2.getWriteLock().lock();
            readWriteRunnableSegment2.getWriteBuffer()[0] = readWriteRunnableSegment.getWriteBuffer()[31];
        }
        finally {
            readWriteRunnableSegment.getWriteLock().unlock();
            readWriteRunnableSegment2.getWriteLock().unlock();
        }
        this.readWriteRunnable.queueSegmentWrite(physicalPort + 6);
    }

    public boolean isI2cPortInReadMode(int physicalPort) {
        this.a(physicalPort);
        boolean bl = false;
        try {
            this.a[physicalPort].getReadLock().lock();
            byte[] arrby = this.a[physicalPort].getReadBuffer();
            bl = arrby[0] == -127;
        }
        finally {
            this.a[physicalPort].getReadLock().unlock();
        }
        return bl;
    }

    public boolean isI2cPortInWriteMode(int physicalPort) {
        this.a(physicalPort);
        boolean bl = false;
        try {
            this.a[physicalPort].getReadLock().lock();
            byte[] arrby = this.a[physicalPort].getReadBuffer();
            bl = arrby[0] == 1;
        }
        finally {
            this.a[physicalPort].getReadLock().unlock();
        }
        return bl;
    }

    public boolean isI2cPortReady(int physicalPort) {
        byte by = this.read(3);
        return this.a(physicalPort, by);
    }

    private void a(int n) {
        if (n < 0 || n > 5) {
            throw new IllegalArgumentException(String.format("port %d is invalid; valid ports are %d..%d", n, Byte.valueOf(0), Byte.valueOf(5)));
        }
    }

    private void b(int n) {
        if (n < 0 || n > 27) {
            throw new IllegalArgumentException(String.format("buffer length of %d is invalid; max value is %d", n, Byte.valueOf(27)));
        }
    }

    private void c(int n) {
        if (n != 0 && n != 1) {
            throw new IllegalArgumentException("line is invalid, valid lines are 0 and 1");
        }
    }

    @Override
    public void readComplete() throws InterruptedException {
        if (this.b == null) {
            return;
        }
        byte by = this.read(3);
        for (int i = 0; i < 6; ++i) {
            if (this.b[i] == null || !this.a(i, by)) continue;
            this.b[i].portIsReady(i);
        }
    }

    private boolean a(int n, byte by) {
        return (by & BUFFER_FLAG_MAP[n]) == 0;
    }

    public Lock getI2cReadCacheLock(int physicalPort) {
        this.a(physicalPort);
        return this.a[physicalPort].getReadLock();
    }

    public Lock getI2cWriteCacheLock(int physicalPort) {
        this.a(physicalPort);
        return this.a[physicalPort].getWriteLock();
    }

    public byte[] getI2cReadCache(int physicalPort) {
        this.a(physicalPort);
        return this.a[physicalPort].getReadBuffer();
    }

    public byte[] getI2cWriteCache(int physicalPort) {
        this.a(physicalPort);
        return this.a[physicalPort].getWriteBuffer();
    }

    @Deprecated
    public void readI2cCacheFromModule(int port) {
        this.readI2cCacheFromController(port);
    }

    @Deprecated
    public void writeI2cCacheToModule(int port) {
        this.writeI2cCacheToController(port);
    }

    @Deprecated
    public void writeI2cPortFlagOnlyToModule(int port) {
        this.writeI2cPortFlagOnlyToController(port);
    }
}

