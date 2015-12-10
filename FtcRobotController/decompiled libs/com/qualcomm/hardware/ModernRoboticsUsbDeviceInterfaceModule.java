/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.eventloop.EventLoopManager
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.DeviceInterfaceModule
 *  com.qualcomm.robotcore.hardware.DigitalChannelController
 *  com.qualcomm.robotcore.hardware.DigitalChannelController$Mode
 *  com.qualcomm.robotcore.hardware.I2cController
 *  com.qualcomm.robotcore.hardware.I2cController$I2cPortReadyCallback
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
 *  com.qualcomm.robotcore.util.ElapsedTime
 *  com.qualcomm.robotcore.util.SerialNumber
 *  com.qualcomm.robotcore.util.TypeConversion
 */
package com.qualcomm.hardware;

import com.qualcomm.hardware.ModernRoboticsUsbDevice;
import com.qualcomm.hardware.ReadWriteRunnable;
import com.qualcomm.hardware.ReadWriteRunnableSegment;
import com.qualcomm.hardware.ReadWriteRunnableStandard;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.TypeConversion;
import java.nio.ByteOrder;
import java.util.concurrent.locks.Lock;

public class ModernRoboticsUsbDeviceInterfaceModule
extends ModernRoboticsUsbDevice
implements DeviceInterfaceModule {
    public static final boolean DEBUG_LOGGING = false;
    public static final int MIN_I2C_PORT_NUMBER = 0;
    public static final int MAX_I2C_PORT_NUMBER = 5;
    public static final int MAX_ANALOG_PORT_NUMBER = 7;
    public static final int MIN_ANALOG_PORT_NUMBER = 0;
    public static final int NUMBER_OF_PORTS = 6;
    public static final int START_ADDRESS = 3;
    public static final int MONITOR_LENGTH = 21;
    public static final int SIZE_I2C_BUFFER = 27;
    public static final int SIZE_ANALOG_BUFFER = 2;
    public static final int WORD_SIZE = 2;
    public static final int ADDRESS_BUFFER_STATUS = 3;
    public static final int ADDRESS_ANALOG_PORT_A0 = 4;
    public static final int ADDRESS_ANALOG_PORT_A1 = 6;
    public static final int ADDRESS_ANALOG_PORT_A2 = 8;
    public static final int ADDRESS_ANALOG_PORT_A3 = 10;
    public static final int ADDRESS_ANALOG_PORT_A4 = 12;
    public static final int ADDRESS_ANALOG_PORT_A5 = 14;
    public static final int ADDRESS_ANALOG_PORT_A6 = 16;
    public static final int ADDRESS_ANALOG_PORT_A7 = 18;
    public static final int ADDRESS_DIGITAL_INPUT_STATE = 20;
    public static final int ADDRESS_DIGITAL_IO_CONTROL = 21;
    public static final int ADDRESS_DIGITAL_OUTPUT_STATE = 22;
    public static final int ADDRESS_LED_SET = 23;
    public static final int ADDRESS_VOLTAGE_OUTPUT_PORT_0 = 24;
    public static final int ADDRESS_VOLTAGE_OUTPUT_PORT_1 = 30;
    public static final int ADDRESS_PULSE_OUTPUT_PORT_0 = 36;
    public static final int ADDRESS_PULSE_OUTPUT_PORT_1 = 40;
    public static final int ADDRESS_I2C0 = 48;
    public static final int ADDRESS_I2C1 = 80;
    public static final int ADDRESS_I2C2 = 112;
    public static final int ADDRESS_I2C3 = 144;
    public static final int ADDRESS_I2C4 = 176;
    public static final int ADDRESS_I2C5 = 208;
    public static final byte BUFFER_FLAG_I2C0 = 1;
    public static final byte BUFFER_FLAG_I2C1 = 2;
    public static final byte BUFFER_FLAG_I2C2 = 4;
    public static final byte BUFFER_FLAG_I2C3 = 8;
    public static final byte BUFFER_FLAG_I2C4 = 16;
    public static final byte BUFFER_FLAG_I2C5 = 32;
    public static final int OFFSET_ANALOG_VOLTAGE_OUTPUT_VOLTAGE = 0;
    public static final int OFFSET_ANALOG_VOLTAGE_OUTPUT_FREQ = 2;
    public static final int OFFSET_ANALOG_VOLTAGE_OUTPUT_MODE = 4;
    public static final int ANALOG_VOLTAGE_OUTPUT_BUFFER_SIZE = 5;
    public static final int OFFSET_PULSE_OUTPUT_TIME = 0;
    public static final int OFFSET_PULSE_OUTPUT_PERIOD = 2;
    public static final int PULSE_OUTPUT_BUFFER_SIZE = 4;
    public static final int OFFSET_I2C_PORT_MODE = 0;
    public static final int OFFSET_I2C_PORT_I2C_ADDRESS = 1;
    public static final int OFFSET_I2C_PORT_MEMORY_ADDRESS = 2;
    public static final int OFFSET_I2C_PORT_MEMORY_LENGTH = 3;
    public static final int OFFSET_I2C_PORT_MEMORY_BUFFER = 4;
    public static final int OFFSET_I2C_PORT_FLAG = 31;
    public static final int I2C_PORT_BUFFER_SIZE = 32;
    public static final byte I2C_MODE_READ = -128;
    public static final byte I2C_MODE_WRITE = 0;
    public static final byte I2C_ACTION_FLAG = -1;
    public static final byte I2C_NO_ACTION_FLAG = 0;
    public static final int LED_0_BIT_MASK = 1;
    public static final int LED_1_BIT_MASK = 2;
    public static final int[] LED_BIT_MASK_MAP = new int[]{1, 2};
    public static final int D0_MASK = 1;
    public static final int D1_MASK = 2;
    public static final int D2_MASK = 4;
    public static final int D3_MASK = 8;
    public static final int D4_MASK = 16;
    public static final int D5_MASK = 32;
    public static final int D6_MASK = 64;
    public static final int D7_MASK = 128;
    public static final int[] ADDRESS_DIGITAL_BIT_MASK = new int[]{1, 2, 4, 8, 16, 32, 64, 128};
    public static final int[] ADDRESS_ANALOG_PORT_MAP = new int[]{4, 6, 8, 10, 12, 14, 16, 18};
    public static final int[] ADDRESS_VOLTAGE_OUTPUT_PORT_MAP = new int[]{24, 30};
    public static final int[] ADDRESS_PULSE_OUTPUT_PORT_MAP = new int[]{36, 40};
    public static final int[] ADDRESS_I2C_PORT_MAP = new int[]{48, 80, 112, 144, 176, 208};
    public static final int[] BUFFER_FLAG_MAP = new int[]{1, 2, 4, 8, 16, 32};
    private static final int[] a = new int[]{0, 1};
    private static final int[] b = new int[]{2, 3};
    private static final int[] c = new int[]{4, 5, 6, 7, 8, 9};
    private static final int[] d = new int[]{10, 11, 12, 13, 14, 15};
    private final I2cController.I2cPortReadyCallback[] e = new I2cController.I2cPortReadyCallback[6];
    private final ElapsedTime[] f = new ElapsedTime[6];
    private ReadWriteRunnableSegment[] g = new ReadWriteRunnableSegment[a.length];
    private ReadWriteRunnableSegment[] h = new ReadWriteRunnableSegment[b.length];
    private ReadWriteRunnableSegment[] i = new ReadWriteRunnableSegment[c.length];
    private ReadWriteRunnableSegment[] j = new ReadWriteRunnableSegment[d.length];

    protected ModernRoboticsUsbDeviceInterfaceModule(SerialNumber serialNumber, RobotUsbDevice device, EventLoopManager manager) throws RobotCoreException, InterruptedException {
        int n;
        super(serialNumber, manager, new ReadWriteRunnableStandard(serialNumber, device, 21, 3, false));
        for (n = 0; n < a.length; ++n) {
            this.g[n] = this.readWriteRunnable.createSegment(a[n], ADDRESS_VOLTAGE_OUTPUT_PORT_MAP[n], 5);
        }
        for (n = 0; n < b.length; ++n) {
            this.h[n] = this.readWriteRunnable.createSegment(b[n], ADDRESS_PULSE_OUTPUT_PORT_MAP[n], 4);
        }
        for (n = 0; n < c.length; ++n) {
            this.i[n] = this.readWriteRunnable.createSegment(c[n], ADDRESS_I2C_PORT_MAP[n], 32);
            this.j[n] = this.readWriteRunnable.createSegment(d[n], ADDRESS_I2C_PORT_MAP[n] + 31, 1);
        }
    }

    @Override
    public String getDeviceName() {
        return "Modern Robotics USB Device Interface Module";
    }

    public String getConnectionInfo() {
        return "USB " + (Object)this.getSerialNumber();
    }

    public int getAnalogInputValue(int channel) {
        this.d(channel);
        byte[] arrby = this.read(ADDRESS_ANALOG_PORT_MAP[channel], 2);
        return TypeConversion.byteArrayToShort((byte[])arrby, (ByteOrder)ByteOrder.LITTLE_ENDIAN);
    }

    public DigitalChannelController.Mode getDigitalChannelMode(int channel) {
        return this.a(channel, (int)this.getDigitalIOControlByte());
    }

    public void setDigitalChannelMode(int channel, DigitalChannelController.Mode mode) {
        int n = this.a(channel, mode);
        byte by = this.readFromWriteCache(21);
        int n2 = mode == DigitalChannelController.Mode.OUTPUT ? by | n : by & n;
        this.write(21, n2);
    }

    public boolean getDigitalChannelState(int channel) {
        int n = DigitalChannelController.Mode.OUTPUT == this.getDigitalChannelMode(channel) ? this.getDigitalOutputStateByte() : this.getDigitalInputStateByte();
        return (n&=ADDRESS_DIGITAL_BIT_MASK[channel]) > 0;
    }

    public void setDigitalChannelState(int channel, boolean state) {
        if (DigitalChannelController.Mode.OUTPUT == this.getDigitalChannelMode(channel)) {
            int n = this.readFromWriteCache(22);
            n = state ? (n|=ADDRESS_DIGITAL_BIT_MASK[channel]) : (n&=~ ADDRESS_DIGITAL_BIT_MASK[channel]);
            this.setDigitalOutputByte((byte)n);
        }
    }

    public int getDigitalInputStateByte() {
        byte by = this.read(20);
        return TypeConversion.unsignedByteToInt((byte)by);
    }

    public byte getDigitalIOControlByte() {
        return this.read(21);
    }

    public void setDigitalIOControlByte(byte input) {
        this.write(21, input);
    }

    public byte getDigitalOutputStateByte() {
        return this.read(22);
    }

    public void setDigitalOutputByte(byte input) {
        this.write(22, input);
    }

    private int a(int n, DigitalChannelController.Mode mode) {
        if (mode == DigitalChannelController.Mode.OUTPUT) {
            return ADDRESS_DIGITAL_BIT_MASK[n];
        }
        return ~ ADDRESS_DIGITAL_BIT_MASK[n];
    }

    private DigitalChannelController.Mode a(int n, int n2) {
        int n3 = ADDRESS_DIGITAL_BIT_MASK[n] & n2;
        if (n3 > 0) {
            return DigitalChannelController.Mode.OUTPUT;
        }
        return DigitalChannelController.Mode.INPUT;
    }

    public boolean getLEDState(int channel) {
        this.a(channel);
        byte by = this.read(23);
        int n = by & LED_BIT_MASK_MAP[channel];
        return n > 0;
    }

    public void setLED(int channel, boolean set) {
        this.a(channel);
        byte by = this.readFromWriteCache(23);
        int n = set ? by | LED_BIT_MASK_MAP[channel] : by & ~ LED_BIT_MASK_MAP[channel];
        this.write(23, n);
    }

    public void setAnalogOutputVoltage(int port, int voltage) {
        this.b(port);
        Lock lock = this.g[port].getWriteLock();
        byte[] arrby = this.g[port].getWriteBuffer();
        byte[] arrby2 = TypeConversion.shortToByteArray((short)((short)voltage), (ByteOrder)ByteOrder.LITTLE_ENDIAN);
        try {
            lock.lock();
            System.arraycopy(arrby2, 0, arrby, 0, arrby2.length);
        }
        finally {
            lock.unlock();
        }
        this.readWriteRunnable.queueSegmentWrite(a[port]);
    }

    public void setAnalogOutputFrequency(int port, int freq) {
        this.b(port);
        Lock lock = this.g[port].getWriteLock();
        byte[] arrby = this.g[port].getWriteBuffer();
        byte[] arrby2 = TypeConversion.shortToByteArray((short)((short)freq), (ByteOrder)ByteOrder.LITTLE_ENDIAN);
        try {
            lock.lock();
            System.arraycopy(arrby2, 0, arrby, 2, arrby2.length);
        }
        finally {
            lock.unlock();
        }
        this.readWriteRunnable.queueSegmentWrite(a[port]);
    }

    public void setAnalogOutputMode(int port, byte mode) {
        this.b(port);
        Lock lock = this.g[port].getWriteLock();
        byte[] arrby = this.g[port].getWriteBuffer();
        try {
            lock.lock();
            arrby[4] = mode;
        }
        finally {
            lock.unlock();
        }
        this.readWriteRunnable.queueSegmentWrite(a[port]);
    }

    public void setPulseWidthOutputTime(int port, int time) {
        this.c(port);
        Lock lock = this.h[port].getWriteLock();
        byte[] arrby = this.h[port].getWriteBuffer();
        byte[] arrby2 = TypeConversion.shortToByteArray((short)((short)time), (ByteOrder)ByteOrder.LITTLE_ENDIAN);
        try {
            lock.lock();
            System.arraycopy(arrby2, 0, arrby, 0, arrby2.length);
        }
        finally {
            lock.unlock();
        }
        this.readWriteRunnable.queueSegmentWrite(b[port]);
    }

    public void setPulseWidthPeriod(int port, int period) {
        this.e(port);
        Lock lock = this.h[port].getWriteLock();
        byte[] arrby = this.h[port].getWriteBuffer();
        byte[] arrby2 = TypeConversion.shortToByteArray((short)((short)period), (ByteOrder)ByteOrder.LITTLE_ENDIAN);
        try {
            lock.lock();
            System.arraycopy(arrby2, 0, arrby, 2, arrby2.length);
        }
        finally {
            lock.unlock();
        }
        this.readWriteRunnable.queueSegmentWrite(b[port]);
    }

    public int getPulseWidthOutputTime(int port) {
        throw new UnsupportedOperationException("getPulseWidthOutputTime is not implemented.");
    }

    public int getPulseWidthPeriod(int port) {
        throw new UnsupportedOperationException("getPulseWidthOutputTime is not implemented.");
    }

    public void enableI2cReadMode(int physicalPort, int i2cAddress, int memAddress, int length) {
        this.e(physicalPort);
        this.f(length);
        try {
            this.i[physicalPort].getWriteLock().lock();
            byte[] arrby = this.i[physicalPort].getWriteBuffer();
            arrby[0] = -128;
            arrby[1] = (byte)i2cAddress;
            arrby[2] = (byte)memAddress;
            arrby[3] = (byte)length;
        }
        finally {
            this.i[physicalPort].getWriteLock().unlock();
        }
    }

    public void enableI2cWriteMode(int physicalPort, int i2cAddress, int memAddress, int length) {
        this.e(physicalPort);
        this.f(length);
        try {
            this.i[physicalPort].getWriteLock().lock();
            byte[] arrby = this.i[physicalPort].getWriteBuffer();
            arrby[0] = 0;
            arrby[1] = (byte)i2cAddress;
            arrby[2] = (byte)memAddress;
            arrby[3] = (byte)length;
        }
        finally {
            this.i[physicalPort].getWriteLock().unlock();
        }
    }

    public byte[] getCopyOfReadBuffer(int physicalPort) {
        this.e(physicalPort);
        Object object = null;
        try {
            this.i[physicalPort].getReadLock().lock();
            byte[] arrby = this.i[physicalPort].getReadBuffer();
            byte by = arrby[3];
            object = new byte[by];
            System.arraycopy(arrby, 4, object, 0, object.length);
        }
        finally {
            this.i[physicalPort].getReadLock().unlock();
        }
        return object;
    }

    public byte[] getCopyOfWriteBuffer(int physicalPort) {
        this.e(physicalPort);
        Object object = null;
        try {
            this.i[physicalPort].getWriteLock().lock();
            byte[] arrby = this.i[physicalPort].getWriteBuffer();
            byte by = arrby[3];
            object = new byte[by];
            System.arraycopy(arrby, 4, object, 0, object.length);
        }
        finally {
            this.i[physicalPort].getWriteLock().unlock();
        }
        return object;
    }

    public void copyBufferIntoWriteBuffer(int physicalPort, byte[] buffer) {
        this.e(physicalPort);
        this.f(buffer.length);
        try {
            this.i[physicalPort].getWriteLock().lock();
            byte[] arrby = this.i[physicalPort].getWriteBuffer();
            System.arraycopy(buffer, 0, arrby, 4, buffer.length);
        }
        finally {
            this.i[physicalPort].getWriteLock().unlock();
        }
    }

    public void setI2cPortActionFlag(int port) {
        this.e(port);
        try {
            this.i[port].getWriteLock().lock();
            byte[] arrby = this.i[port].getWriteBuffer();
            arrby[31] = -1;
        }
        finally {
            this.i[port].getWriteLock().unlock();
        }
    }

    public boolean isI2cPortActionFlagSet(int port) {
        this.e(port);
        boolean bl = false;
        try {
            this.i[port].getReadLock().lock();
            byte[] arrby = this.i[port].getReadBuffer();
            bl = arrby[31] == -1;
        }
        finally {
            this.i[port].getReadLock().unlock();
        }
        return bl;
    }

    public void readI2cCacheFromController(int port) {
        this.e(port);
        this.readWriteRunnable.queueSegmentRead(c[port]);
    }

    public void writeI2cCacheToController(int port) {
        this.e(port);
        this.readWriteRunnable.queueSegmentWrite(c[port]);
    }

    public void writeI2cPortFlagOnlyToController(int port) {
        this.e(port);
        ReadWriteRunnableSegment readWriteRunnableSegment = this.i[port];
        ReadWriteRunnableSegment readWriteRunnableSegment2 = this.j[port];
        try {
            readWriteRunnableSegment.getWriteLock().lock();
            readWriteRunnableSegment2.getWriteLock().lock();
            readWriteRunnableSegment2.getWriteBuffer()[0] = readWriteRunnableSegment.getWriteBuffer()[31];
        }
        finally {
            readWriteRunnableSegment.getWriteLock().unlock();
            readWriteRunnableSegment2.getWriteLock().unlock();
        }
        this.readWriteRunnable.queueSegmentWrite(d[port]);
    }
    
    public boolean isI2cPortInReadMode(int port) {
        this.e(port);
        boolean bl = false;
        try {
            this.i[port].getReadLock().lock();
            byte[] arrby = this.i[port].getReadBuffer();
            bl = arrby[0] == -128;
        }
        finally {
            this.i[port].getReadLock().unlock();
        }
        return bl;
    }

    public boolean isI2cPortInWriteMode(int port) {
        this.e(port);
        boolean bl = false;
        try {
            this.i[port].getReadLock().lock();
            byte[] arrby = this.i[port].getReadBuffer();
            bl = arrby[0] == 0;
        }
        finally {
            this.i[port].getReadLock().unlock();
        }
        return bl;
    }

    public boolean isI2cPortReady(int port) {
        return this.a(port, this.read(3));
    }

    public Lock getI2cReadCacheLock(int port) {
        return this.i[port].getReadLock();
    }

    public Lock getI2cWriteCacheLock(int port) {
        return this.i[port].getWriteLock();
    }

    public byte[] getI2cReadCache(int port) {
        return this.i[port].getReadBuffer();
    }

    public byte[] getI2cWriteCache(int port) {
        return this.i[port].getWriteBuffer();
    }

    public void registerForI2cPortReadyCallback(I2cController.I2cPortReadyCallback callback, int port) {
        this.e[port] = callback;
    }

    public void deregisterForPortReadyCallback(int port) {
        this.e[port] = null;
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

    private void a(int n) {
        if (n != 0 && n != 1) {
            throw new IllegalArgumentException(String.format("port %d is invalid; valid ports are 0 and 1.", n));
        }
    }

    private void b(int n) {
        if (n != 0 && n != 1) {
            throw new IllegalArgumentException(String.format("port %d is invalid; valid ports are 0 and 1.", n));
        }
    }

    private void c(int n) {
        if (n != 0 && n != 1) {
            throw new IllegalArgumentException(String.format("port %d is invalid; valid ports are 0 and 1.", n));
        }
    }

    private void d(int n) {
        if (n < 0 || n > 7) {
            throw new IllegalArgumentException(String.format("port %d is invalid; valid ports are %d..%d", n, 0, 7));
        }
    }

    private void e(int n) {
        if (n < 0 || n > 5) {
            throw new IllegalArgumentException(String.format("port %d is invalid; valid ports are %d..%d", n, 0, 5));
        }
    }

    private void f(int n) {
        if (n > 27) {
            throw new IllegalArgumentException(String.format("buffer is too large (%d byte), max size is %d bytes", n, 27));
        }
    }

    private boolean a(int n, byte by) {
        return (by & BUFFER_FLAG_MAP[n]) == 0;
    }

    @Override
    public void readComplete() throws InterruptedException {
        if (this.e == null) {
            return;
        }
        byte by = this.read(3);
        for (int i = 0; i < 6; ++i) {
            if (this.e[i] == null || !this.a(i, by)) continue;
            this.e[i].portIsReady(i);
        }
    }
}

