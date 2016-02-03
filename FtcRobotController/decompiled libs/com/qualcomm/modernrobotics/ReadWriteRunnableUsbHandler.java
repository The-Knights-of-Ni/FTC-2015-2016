/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice$Channel
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.Util
 */
package com.qualcomm.modernrobotics;

import com.qualcomm.modernrobotics.a;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Util;
import java.util.Arrays;

public class ReadWriteRunnableUsbHandler {
    protected final int MAX_SEQUENTIAL_USB_ERROR_COUNT = 10;
    protected final int USB_MSG_TIMEOUT = 100;
    protected int usbSequentialReadErrorCount = 0;
    protected int usbSequentialWriteErrorCount = 0;
    protected RobotUsbDevice device;
    protected final byte[] respHeader = new byte[5];
    protected byte[] writeCmd = new byte[]{85, -86, 0, 0, 0};
    protected byte[] readCmd = new byte[]{85, -86, -128, 0, 0};

    public ReadWriteRunnableUsbHandler(RobotUsbDevice device) {
        this.device = device;
    }

    public void throwIfUsbErrorCountIsTooHigh() throws RobotCoreException {
        if (this.usbSequentialReadErrorCount > 10 || this.usbSequentialWriteErrorCount > 10) {
            throw new RobotCoreException("Too many sequential USB errors on device");
        }
    }

    public void read(int address, byte[] buffer) throws RobotCoreException, InterruptedException {
        this.a(address, buffer);
    }

    private void a(int n, byte[] arrby) throws RobotCoreException, InterruptedException {
        this.readCmd[3] = (byte)n;
        this.readCmd[4] = (byte)arrby.length;
        this.device.write(this.readCmd);
        Arrays.fill(this.respHeader, 0);
        int n2 = this.device.read(this.respHeader, this.respHeader.length, 100);
        if (!a.a(this.respHeader, arrby.length)) {
            ++this.usbSequentialReadErrorCount;
            if (n2 == this.respHeader.length) {
                Thread.sleep(100);
                this.a(this.readCmd, "comm error");
            } else {
                this.a(this.readCmd, "comm timeout");
            }
        }
        if ((n2 = this.device.read(arrby, arrby.length, 100)) != arrby.length) {
            this.a(this.readCmd, "comm timeout on payload");
        }
        this.usbSequentialReadErrorCount = 0;
    }

    public void write(int address, byte[] buffer) throws RobotCoreException, InterruptedException {
        this.b(address, buffer);
    }

    private void b(int n, byte[] arrby) throws RobotCoreException, InterruptedException {
        this.writeCmd[3] = (byte)n;
        this.writeCmd[4] = (byte)arrby.length;
        this.device.write(Util.concatenateByteArrays((byte[])this.writeCmd, (byte[])arrby));
        Arrays.fill(this.respHeader, 0);
        int n2 = this.device.read(this.respHeader, this.respHeader.length, 100);
        if (!a.a(this.respHeader, 0)) {
            ++this.usbSequentialWriteErrorCount;
            if (n2 == this.respHeader.length) {
                Thread.sleep(100);
                this.a(this.writeCmd, "comm error");
            } else {
                this.a(this.writeCmd, "comm timeout");
            }
        }
        this.usbSequentialWriteErrorCount = 0;
    }

    public void purge(RobotUsbDevice.Channel channel) throws RobotCoreException {
        this.device.purge(channel);
    }

    public void close() {
        this.device.close();
    }

    private void a(byte[] arrby, String string) throws RobotCoreException {
        RobotLog.w((String)(ReadWriteRunnableUsbHandler.bufferToString(arrby) + " -> " + ReadWriteRunnableUsbHandler.bufferToString(this.respHeader)));
        this.device.purge(RobotUsbDevice.Channel.BOTH);
        throw new RobotCoreException(string);
    }

    protected static String bufferToString(byte[] buffer) {
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append("[");
        if (buffer.length > 0) {
            stringBuilder.append(String.format("%02x", Byte.valueOf(buffer[0])));
        }
        for (int i = 1; i < buffer.length; ++i) {
            stringBuilder.append(String.format(" %02x", Byte.valueOf(buffer[i])));
        }
        stringBuilder.append("]");
        return stringBuilder.toString();
    }
}

