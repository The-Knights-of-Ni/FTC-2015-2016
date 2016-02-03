/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.hardware.usb.ftdi;

import com.ftdi.j2xx.FT_Device;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;

public class RobotUsbDeviceFtdi
implements RobotUsbDevice {
    private FT_Device a;

    public RobotUsbDeviceFtdi(FT_Device device) {
        this.a = device;
    }

    @Override
    public void setBaudRate(int rate) throws RobotCoreException {
        if (!this.a.setBaudRate(rate)) {
            throw new RobotCoreException("FTDI driver failed to set baud rate to " + rate);
        }
    }

    @Override
    public void setDataCharacteristics(byte dataBits, byte stopBits, byte parity) throws RobotCoreException {
        if (!this.a.setDataCharacteristics(dataBits, stopBits, parity)) {
            throw new RobotCoreException("FTDI driver failed to set data characteristics");
        }
    }

    @Override
    public void setLatencyTimer(int latencyTimer) throws RobotCoreException {
        if (!this.a.setLatencyTimer((byte)latencyTimer)) {
            throw new RobotCoreException("FTDI driver failed to set latency timer to " + latencyTimer);
        }
    }

    @Override
    public void purge(RobotUsbDevice.Channel channel) throws RobotCoreException {
        byte by = 0;
        switch (channel) {
            case RX: {
                by = 1;
                break;
            }
            case TX: {
                by = 2;
                break;
            }
            case BOTH: {
                by = 3;
            }
        }
        this.a.purge(by);
    }

    @Override
    public void write(byte[] data) throws RobotCoreException {
        this.a.write(data);
    }

    @Override
    public int read(byte[] data) throws RobotCoreException {
        return this.a.read(data);
    }

    @Override
    public int read(byte[] data, int length, int timeout) throws RobotCoreException {
        return this.a.read(data, length, timeout);
    }

    @Override
    public void close() {
        this.a.close();
    }

}

