/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice$Channel
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.SerialNumber
 *  com.qualcomm.robotcore.util.TypeConversion
 */
package com.qualcomm.modernrobotics;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.TypeConversion;
import java.util.Arrays;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;

class b
implements RobotUsbDevice {
    public final boolean a;
    public SerialNumber b;
    public String c;
    private byte[] f;
    private byte[] g;
    private BlockingQueue<byte[]> h;
    protected final byte[] d;
    protected final byte[] e;

    public void setBaudRate(int rate) throws RobotCoreException {
    }

    public void setDataCharacteristics(byte dataBits, byte stopBits, byte parity) throws RobotCoreException {
    }

    public void setLatencyTimer(int latencyTimer) throws RobotCoreException {
    }

    public void purge(RobotUsbDevice.Channel channel) throws RobotCoreException {
        this.h.clear();
    }

    public void write(byte[] data) throws RobotCoreException {
        this.a(data);
    }

    public int read(byte[] data) throws RobotCoreException {
        return this.read(data, data.length, Integer.MAX_VALUE);
    }

    public int read(byte[] data, int length, int timeout) throws RobotCoreException {
        return this.a(data, length, timeout);
    }

    public void close() {
    }

    private void a(final byte[] arrby) {
        if (this.a) {
            RobotLog.d((String)((Object)this.b + " USB recd: " + Arrays.toString(arrby)));
        }
        new Thread(){

            @Override
            public void run() {
                int n = TypeConversion.unsignedByteToInt((byte)arrby[3]);
                int n2 = TypeConversion.unsignedByteToInt((byte)arrby[4]);
                try {
                    byte[] arrby2;
                    Thread.sleep(10);
                    switch (arrby[2]) {
                        case -128: {
                            arrby2 = new byte[b.this.e.length + n2];
                            System.arraycopy(b.this.e, 0, arrby2, 0, b.this.e.length);
                            arrby2[3] = arrby[3];
                            arrby2[4] = arrby[4];
                            System.arraycopy(b.this.f, n, arrby2, b.this.e.length, n2);
                            break;
                        }
                        case 0: {
                            arrby2 = new byte[b.this.d.length];
                            System.arraycopy(b.this.d, 0, arrby2, 0, b.this.d.length);
                            arrby2[3] = arrby[3];
                            arrby2[4] = 0;
                            System.arraycopy(arrby, 5, b.this.f, n, n2);
                            break;
                        }
                        default: {
                            arrby2 = Arrays.copyOf(arrby, arrby.length);
                            arrby2[2] = -1;
                            arrby2[3] = arrby[3];
                            arrby2[4] = 0;
                        }
                    }
                    b.this.h.put(arrby2);
                }
                catch (InterruptedException var3_4) {
                    RobotLog.w((String)"USB mock bus interrupted during write");
                }
            }
        }.start();
    }

    private int a(byte[] arrby, int n, int n2) {
        Object object = null;
        if (this.g != null) {
            object = Arrays.copyOf(this.g, this.g.length);
            this.g = null;
        } else {
            try {
                object = this.h.poll(n2, TimeUnit.MILLISECONDS);
            }
            catch (InterruptedException var5_5) {
                RobotLog.w((String)"USB mock bus interrupted during read");
            }
        }
        if (object == null) {
            RobotLog.w((String)"USB mock bus read timeout");
            System.arraycopy(this.e, 0, arrby, 0, this.e.length);
            arrby[2] = -1;
            arrby[4] = 0;
        } else {
            System.arraycopy(object, 0, arrby, 0, n);
        }
        if (object != null && n < object.length) {
            this.g = new byte[object.length - n];
            System.arraycopy(object, arrby.length, this.g, 0, this.g.length);
        }
        if (this.a) {
            RobotLog.d((String)((Object)this.b + " USB send: " + Arrays.toString(arrby)));
        }
        return arrby.length;
    }

}

