/*
 * Decompiled with CFR 0_101.
 */
package com.ftdi.j2xx.ft4222;

import com.ftdi.j2xx.FT_Device;
import com.ftdi.j2xx.ft4222.FT_4222_Device;
import com.ftdi.j2xx.ft4222.b;
import com.ftdi.j2xx.interfaces.I2cSlave;

public class FT_4222_I2c_Slave
implements I2cSlave {
    FT_4222_Device a;
    FT_Device b;

    public FT_4222_I2c_Slave(FT_4222_Device ft4222Device) {
        this.a = ft4222Device;
        this.b = this.a.mFtDev;
    }

    int a(int n, int n2) {
        return this.b.VendorCmdSet(33, n | n2 << 8);
    }

    public int init() {
        int n = this.a.init();
        if (n != 0) {
            return n;
        }
        if (!this.a()) {
            return 1012;
        }
        n = this.a(5, 2);
        if (n < 0) {
            return n;
        }
        this.a.mChipStatus.g = 2;
        return 0;
    }

    public int reset() {
        int n = 1;
        int n2 = this.a(false);
        if (n2 != 0) {
            return n2;
        }
        return this.a(91, n);
    }

    public int getAddress(int[] addr) {
        byte[] arrby = new byte[1];
        int n = this.a(false);
        if (n != 0) {
            return n;
        }
        n = this.b.VendorCmdGet(33, 92, arrby, 1);
        if (n < 0) {
            return 18;
        }
        addr[0] = arrby[0];
        return 0;
    }

    public int setAddress(int addr) {
        byte[] arrby = new byte[]{(byte)(addr & 255)};
        int n = this.a(false);
        if (n != 0) {
            return n;
        }
        n = this.a(92, arrby[0]);
        if (n < 0) {
            return 18;
        }
        return 0;
    }

    public int read(byte[] buffer, int sizeToTransfer, int[] sizeTransferred) {
        int[] arrn = new int[1];
        long l = System.currentTimeMillis();
        int n = this.b.getReadTimeout();
        if (sizeToTransfer < 1) {
            return 6;
        }
        int n2 = this.a(false);
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
        int n3 = this.b.getQueueStatus();
        while (n3 < sizeToTransfer && System.currentTimeMillis() - l < (long)n) {
            n3 = this.b.getQueueStatus();
        }
        if (n3 > sizeToTransfer) {
            n3 = sizeToTransfer;
        }
        if ((n2 = this.b.read(buffer, n3)) < 0) {
            return 1011;
        }
        sizeTransferred[0] = n2;
        return 0;
    }

    public int write(byte[] buffer, int sizeToTransfer, int[] sizeTransferred) {
        int[] arrn = new int[1];
        if (sizeToTransfer < 1) {
            return 6;
        }
        int n = this.a(false);
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
        sizeTransferred[0] = n = this.b.write(buffer, sizeToTransfer);
        if (sizeToTransfer == n) {
            return 0;
        }
        return 10;
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

    int a(int[] arrn) {
        arrn[0] = 0;
        int n = this.a.getMaxBuckSize();
        switch (this.a.mChipStatus.g) {
            case 2: {
                arrn[0] = n - 4;
                break;
            }
            default: {
                return 17;
            }
        }
        return 0;
    }
}

