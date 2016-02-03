/*
 * Decompiled with CFR 0_101.
 */
package com.ftdi.j2xx.ft4222;

import com.ftdi.j2xx.FT_Device;
import com.ftdi.j2xx.ft4222.FT_4222_Device;
import com.ftdi.j2xx.ft4222.b;
import com.ftdi.j2xx.interfaces.I2cMaster;

public class FT_4222_I2c_Master
implements I2cMaster {
    FT_4222_Device a;
    FT_Device b;
    int c;

    public FT_4222_I2c_Master(FT_4222_Device ft4222Device) {
        this.a = ft4222Device;
        this.b = this.a.mFtDev;
    }

    int a(int n, int n2) {
        return this.b.VendorCmdSet(33, n | n2 << 8);
    }

    public int init(int kbps) {
        byte[] arrby = new byte[1];
        int n = this.a.init();
        if (n != 0) {
            return n;
        }
        if (!this.a()) {
            return 1012;
        }
        this.a(81, 0);
        n = this.a.getClock(arrby);
        if (n != 0) {
            return n;
        }
        int n2 = this.b(arrby[0], kbps);
        n = this.a(5, 1);
        if (n < 0) {
            return n;
        }
        this.a.mChipStatus.g = 1;
        n = this.a(82, n2);
        if (n < 0) {
            return n;
        }
        this.c = kbps;
        return 0;
    }

    public int reset() {
        int n = 1;
        int n2 = this.a(true);
        if (n2 != 0) {
            return n2;
        }
        return this.a(81, n);
    }

    public int read(int deviceAddress, byte[] buffer, int sizeToTransfer, int[] sizeTransferred) {
        return this.readEx(deviceAddress, 6, buffer, sizeToTransfer, sizeTransferred);
    }

    public int readEx(int deviceAddress, int flag, byte[] buffer, int sizeToTransfer, int[] sizeTransferred) {
        short s = (short)(deviceAddress & 65535);
        short s2 = (short)sizeToTransfer;
        int[] arrn = new int[1];
        byte[] arrby = new byte[4];
        long l = System.currentTimeMillis();
        int n = this.b.getReadTimeout();
        int n2 = this.a(flag);
        if (n2 != 0) {
            return n2;
        }
        n2 = this.b(deviceAddress);
        if (n2 != 0) {
            return n2;
        }
        if (sizeToTransfer < 1) {
            return 6;
        }
        n2 = this.a(true);
        if (n2 != 0) {
            return n2;
        }
        n2 = this.a(arrn);
        if (n2 != 0) {
            return n2;
        }
        if (sizeToTransfer > arrn[0]) {
            return 1010;
        }
        sizeTransferred[0] = 0;
        s = (short)((s << 1) + 1);
        arrby[0] = (byte)s;
        arrby[1] = (byte)flag;
        arrby[2] = (byte)(s2 >> 8 & 255);
        arrby[3] = (byte)(s2 & 255);
        n2 = this.b.write(arrby, 4);
        if (4 != n2) {
            return 1011;
        }
        int n3 = this.b.getQueueStatus();
        while (n3 < sizeToTransfer && System.currentTimeMillis() - l < (long)n) {
            n3 = this.b.getQueueStatus();
        }
        if (n3 > sizeToTransfer) {
            n3 = sizeToTransfer;
        }
        sizeTransferred[0] = n2 = this.b.read(buffer, n3);
        if (n2 >= 0) {
            return 0;
        }
        return 1011;
    }

    public int write(int deviceAddress, byte[] buffer, int sizeToTransfer, int[] sizeTransferred) {
        return this.writeEx(deviceAddress, 6, buffer, sizeToTransfer, sizeTransferred);
    }

    public int writeEx(int deviceAddress, int flag, byte[] buffer, int sizeToTransfer, int[] sizeTransferred) {
        short s = (short)deviceAddress;
        short s2 = (short)sizeToTransfer;
        byte[] arrby = new byte[sizeToTransfer + 4];
        int[] arrn = new int[1];
        int n = this.a(flag);
        if (n != 0) {
            return n;
        }
        n = this.b(deviceAddress);
        if (n != 0) {
            return n;
        }
        if (sizeToTransfer < 1) {
            return 6;
        }
        n = this.a(true);
        if (n != 0) {
            return n;
        }
        n = this.a(arrn);
        if (n != 0) {
            return n;
        }
        if (sizeToTransfer > arrn[0]) {
            return 1010;
        }
        sizeTransferred[0] = 0;
        s = (short)(s << 1);
        arrby[0] = (byte)s;
        arrby[1] = (byte)flag;
        arrby[2] = (byte)(s2 >> 8 & 255);
        arrby[3] = (byte)(s2 & 255);
        for (int i = 0; i < sizeToTransfer; ++i) {
            arrby[i + 4] = buffer[i];
        }
        n = this.b.write(arrby, sizeToTransfer + 4);
        sizeTransferred[0] = n - 4;
        if (sizeToTransfer == sizeTransferred[0]) {
            return 0;
        }
        return 10;
    }

    public int getStatus(int deviceAddress, byte[] controllerStatus) {
        int n = this.a(true);
        if (n != 0) {
            return n;
        }
        n = this.b.VendorCmdGet(34, 62900, controllerStatus, 1);
        if (n < 0) {
            return 18;
        }
        return 0;
    }

    boolean a() {
        if (this.a.mChipStatus.a == 0 || this.a.mChipStatus.a == 3) {
            return true;
        }
        return false;
    }

    int a(boolean bl) {
        if (bl ? this.a.mChipStatus.g != 1 : this.a.mChipStatus.g != 2) {
            return 1004;
        }
        return 0;
    }

    int a(int n) {
        if (!(this.b != null && this.b.isOpen())) {
            return 3;
        }
        if (n != 6) {
            char[] arrc = new char[1];
            this.a(arrc);
            if (arrc[0] < 'B') {
                return 1022;
            }
        }
        return 0;
    }

    int b(int n) {
        if ((n & 64512) > 0) {
            return 1007;
        }
        return 0;
    }

    private int b(int n, int n2) {
        int n3;
        double d;
        switch (n) {
            default: {
                d = 16.666666666666668;
                break;
            }
            case 1: {
                d = 41.666666666666664;
                break;
            }
            case 2: {
                d = 20.833333333333332;
                break;
            }
            case 3: {
                d = 12.5;
            }
        }
        if (60 <= n2 && n2 <= 100) {
            int n4 = 2;
            int n5 = 2;
            double d2 = 1000000.0 / (double)n2;
            int n6 = (int)(d2 / (8.0 * d) - 1.0 + 0.5);
            if (n6 > 127) {
                n6 = 127;
            }
            n3 = n6;
        } else if (100 < n2 && n2 <= 400) {
            int n7;
            int n8 = 2;
            boolean bl = true;
            double d3 = 1000000.0 / (double)n2;
            n3 = n7 = (int)(d3 / (6.0 * d) - 1.0 + 0.5);
            n3|=192;
        } else if (400 < n2 && n2 <= 1000) {
            int n9;
            int n10 = 2;
            boolean bl = true;
            double d4 = 1000000.0 / (double)n2;
            n3 = n9 = (int)(d4 / (6.0 * d) - 1.0 + 0.5);
            n3|=192;
        } else if (1000 < n2 && n2 <= 3400) {
            int n11;
            int n12 = 2;
            boolean bl = true;
            double d5 = 1000000.0 / (double)n2;
            n3 = n11 = (int)(d5 / (6.0 * d) - 1.0 + 0.5);
            n3|=128;
            n3&=-65;
        } else {
            n3 = 74;
        }
        return n3;
    }

    int a(int[] arrn) {
        arrn[0] = 0;
        int n = this.a.getMaxBuckSize();
        switch (this.a.mChipStatus.g) {
            case 1: {
                arrn[0] = n - 4;
                break;
            }
            default: {
                return 17;
            }
        }
        return 0;
    }

    int a(char[] arrc) {
        byte[] arrby = new byte[12];
        int n = this.b.VendorCmdGet(32, 0, arrby, 12);
        if (n < 0) {
            return 18;
        }
        if (arrby[2] == 1) {
            arrc[0] = 65;
        } else if (arrby[2] == 2) {
            arrc[0] = 66;
        }
        return 0;
    }
}

