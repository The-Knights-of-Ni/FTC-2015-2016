/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.util.Log
 */
package com.ftdi.j2xx.ft4222;

import android.util.Log;
import com.ftdi.j2xx.FT_Device;
import com.ftdi.j2xx.ft4222.FT_4222_Device;
import com.ftdi.j2xx.ft4222.b;
import com.ftdi.j2xx.ft4222.c;
import com.ftdi.j2xx.ft4222.d;
import com.ftdi.j2xx.ft4222.e;
import com.ftdi.j2xx.interfaces.Gpio;

public class FT_4222_Gpio
implements Gpio {
    private FT_4222_Device a;
    private FT_Device b;

    public FT_4222_Gpio(FT_4222_Device ft4222Device) {
        this.a = ft4222Device;
        this.b = this.a.mFtDev;
    }

    int a(int n, int n2) {
        return this.b.VendorCmdSet(33, n | n2 << 8);
    }

    int a(int n, int n2, byte[] arrby, int n3) {
        return this.b.VendorCmdGet(32, n | n2 << 8, arrby, n3);
    }

    public int init(int[] gpio) {
        b b = this.a.mChipStatus;
        char[] arrc = new char[1];
        this.a(arrc);
        d d = new d(arrc);
        byte[] arrby = new byte[1];
        e e = new e();
        this.a(7, 0);
        this.a(6, 0);
        int n = this.a.init();
        if (n != 0) {
            Log.e((String)"GPIO_M", (String)("FT4222_GPIO init - 1 NG ftStatus:" + n));
            return n;
        }
        if (b.a == 2 || b.a == 3) {
            return 1013;
        }
        this.a(d);
        byte by = d.c;
        arrby[0] = d.d[0];
        for (int i = 0; i < 4; ++i) {
            by = gpio[i] == 1 ? (byte)((by | 1 << i) & 15) : (byte)(by & ~ (1 << i) & 15);
        }
        e.c = arrby[0];
        this.a(33, by);
        return 0;
    }

    public int read(int portNum, boolean[] bValue) {
        char[] arrc = new char[1];
        this.a(arrc);
        d d = new d(arrc);
        int n = this.a(portNum);
        if (n != 0) {
            return n;
        }
        n = this.a(d);
        if (n != 0) {
            return n;
        }
        this.a(portNum, d.d[0], bValue);
        return 0;
    }

    public int write(int portNum, boolean bValue) {
        char[] arrc = new char[1];
        this.a(arrc);
        d d = new d(arrc);
        int n = this.a(portNum);
        if (n != 0) {
            return n;
        }
        if (!this.c(portNum)) {
            return 1015;
        }
        this.a(d);
        if (bValue) {
            byte[] arrby = d.d;
            arrby[0] = (byte)(arrby[0] | 1 << portNum);
        } else {
            byte[] arrby = d.d;
            arrby[0] = (byte)(arrby[0] & (~ (1 << portNum) & 15));
        }
        int n2 = this.b.write(d.d, 1);
        return n2;
    }

    int a(int n) {
        b b = this.a.mChipStatus;
        if (b.a == 2 || b.a == 3) {
            return 1013;
        }
        if (n >= 4) {
            return 1014;
        }
        return 0;
    }

    int a(d d) {
        char[] arrc = new char[1];
        this.a(arrc);
        byte[] arrby = arrc[0] < 'B' ? new byte[8] : new byte[6];
        int n = this.a(32, 0, arrby, arrby.length);
        d.a.a = arrby[0];
        d.a.b = arrby[1];
        d.b = arrby[arrby.length - 3];
        d.c = arrby[arrby.length - 2];
        d.d[0] = arrby[arrby.length - 1];
        if (n == arrby.length) {
            return 0;
        }
        return n;
    }

    void a(int n, byte by, boolean[] arrbl) {
        arrbl[0] = this.d((by & 1 << n) >> n & 1);
    }

    boolean b(int n) {
        b b = this.a.mChipStatus;
        boolean bl = true;
        switch (b.a) {
            case 0: {
                if ((n == 0 || n == 1) && (b.g == 1 || b.g == 2)) {
                    bl = false;
                }
                if (this.d(b.i) && n == 2) {
                    bl = false;
                }
                if (!this.d(b.j) || n != 3) break;
                bl = false;
                break;
            }
            case 1: {
                if (n == 0 || n == 1) {
                    bl = false;
                }
                if (this.d(b.i) && n == 2) {
                    bl = false;
                }
                if (!this.d(b.j) || n != 3) break;
                bl = false;
                break;
            }
            case 2: 
            case 3: {
                bl = false;
            }
        }
        return bl;
    }

    boolean c(int n) {
        char[] arrc = new char[1];
        this.a(arrc);
        d d = new d(arrc);
        boolean bl = this.b(n);
        this.a(d);
        if (bl && (d.c >> n & 1) != 1) {
            bl = false;
        }
        return bl;
    }

    boolean d(int n) {
        return n != 0;
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

