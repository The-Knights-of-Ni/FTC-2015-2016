/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.hardware.usb.UsbDeviceConnection
 */
package com.ftdi.j2xx;

import android.hardware.usb.UsbDeviceConnection;
import com.ftdi.j2xx.FT_Device;
import com.ftdi.j2xx.FT_EEPROM;
import com.ftdi.j2xx.FT_EEPROM_245R;
import com.ftdi.j2xx.k;

class i
extends k {
    private static FT_Device d;

    i(FT_Device fT_Device) {
        super(fT_Device);
        d = fT_Device;
    }

    boolean a(short s, short s2) {
        int n = s2 & 65535;
        int n2 = s & 65535;
        int n3 = 0;
        boolean bl = false;
        byte by = 0;
        if (s >= 1024) {
            return bl;
        }
        by = d.getLatencyTimer();
        d.setLatencyTimer(119);
        n3 = d.c().controlTransfer(64, 145, n, n2, null, 0, 0);
        if (n3 == 0) {
            bl = true;
        }
        d.setLatencyTimer(by);
        return bl;
    }

    short a(FT_EEPROM fT_EEPROM) {
        int[] arrn = new int[80];
        if (fT_EEPROM.getClass() != FT_EEPROM_245R.class) {
            return 1;
        }
        FT_EEPROM_245R fT_EEPROM_245R = (FT_EEPROM_245R)fT_EEPROM;
        try {
            int n;
            short s;
            for (s = 0; s < 80; s = (short)(s + 1)) {
                arrn[s] = this.a(s);
            }
            int n2 = 0;
            n2|=arrn[0] & 65280;
            if (fT_EEPROM_245R.HighIO) {
                n2|=4;
            }
            if (fT_EEPROM_245R.LoadVCP) {
                n2|=8;
            }
            n2 = fT_EEPROM_245R.ExternalOscillator ? (n2|=2) : (n2&=65533);
            arrn[0] = n2;
            arrn[1] = fT_EEPROM_245R.VendorId;
            arrn[2] = fT_EEPROM_245R.ProductId;
            arrn[3] = 1536;
            arrn[4] = this.a((Object)fT_EEPROM);
            int n3 = this.b(fT_EEPROM);
            if (fT_EEPROM_245R.InvertTXD) {
                n3|=256;
            }
            if (fT_EEPROM_245R.InvertRXD) {
                n3|=512;
            }
            if (fT_EEPROM_245R.InvertRTS) {
                n3|=1024;
            }
            if (fT_EEPROM_245R.InvertCTS) {
                n3|=2048;
            }
            if (fT_EEPROM_245R.InvertDTR) {
                n3|=4096;
            }
            if (fT_EEPROM_245R.InvertDSR) {
                n3|=8192;
            }
            if (fT_EEPROM_245R.InvertDCD) {
                n3|=16384;
            }
            if (fT_EEPROM_245R.InvertRI) {
                n3|=32768;
            }
            arrn[5] = n3;
            int n4 = 0;
            s = fT_EEPROM_245R.CBus0;
            int n5 = fT_EEPROM_245R.CBus1;
            int n6 = fT_EEPROM_245R.CBus2;
            int n7 = fT_EEPROM_245R.CBus3;
            arrn[10] = n4 = s | (n5<<=4) | (n6<<=8) | (n7<<=12);
            int n8 = 0;
            arrn[11] = n8 = (n = fT_EEPROM_245R.CBus4);
            int n9 = 12;
            n9 = this.a(fT_EEPROM_245R.Manufacturer, arrn, n9, 7, true);
            n9 = this.a(fT_EEPROM_245R.Product, arrn, n9, 8, true);
            if (fT_EEPROM_245R.SerNumEnable) {
                n9 = this.a(fT_EEPROM_245R.SerialNumber, arrn, n9, 9, true);
            }
            byte by = 0;
            if (arrn[1] != 0 && arrn[2] != 0) {
                boolean bl = false;
                by = d.getLatencyTimer();
                d.setLatencyTimer(119);
                bl = this.a(arrn, 80);
                d.setLatencyTimer(by);
                if (bl) {
                    return 0;
                }
                return 1;
            }
            return 2;
        }
        catch (Exception var8_5) {
            var8_5.printStackTrace();
            return 0;
        }
    }

    FT_EEPROM a() {
        FT_EEPROM_245R fT_EEPROM_245R = new FT_EEPROM_245R();
        int[] arrn = new int[80];
        try {
            int n;
            for (n = 0; n < 80; ++n) {
                arrn[n] = this.a((short)n);
            }
            fT_EEPROM_245R.HighIO = (arrn[0] & 4) == 4;
            fT_EEPROM_245R.LoadVCP = (arrn[0] & 8) == 8;
            fT_EEPROM_245R.ExternalOscillator = (arrn[0] & 2) == 2;
            fT_EEPROM_245R.VendorId = (short)arrn[1];
            fT_EEPROM_245R.ProductId = (short)arrn[2];
            this.a(fT_EEPROM_245R, arrn[4]);
            this.a((Object)fT_EEPROM_245R, arrn[5]);
            fT_EEPROM_245R.InvertTXD = (arrn[5] & 256) == 256;
            fT_EEPROM_245R.InvertRXD = (arrn[5] & 512) == 512;
            fT_EEPROM_245R.InvertRTS = (arrn[5] & 1024) == 1024;
            fT_EEPROM_245R.InvertCTS = (arrn[5] & 2048) == 2048;
            fT_EEPROM_245R.InvertDTR = (arrn[5] & 4096) == 4096;
            fT_EEPROM_245R.InvertDSR = (arrn[5] & 8192) == 8192;
            fT_EEPROM_245R.InvertDCD = (arrn[5] & 16384) == 16384;
            fT_EEPROM_245R.InvertRI = (arrn[5] & 32768) == 32768;
            n = arrn[10];
            int n2 = n & 15;
            fT_EEPROM_245R.CBus0 = (byte)n2;
            int n3 = n & 240;
            fT_EEPROM_245R.CBus1 = (byte)(n3 >> 4);
            int n4 = n & 3840;
            fT_EEPROM_245R.CBus2 = (byte)(n4 >> 8);
            int n5 = n & 61440;
            fT_EEPROM_245R.CBus3 = (byte)(n5 >> 12);
            int n6 = arrn[11] & 255;
            fT_EEPROM_245R.CBus4 = (byte)n6;
            int n7 = arrn[7] & 255;
            n7-=128;
            fT_EEPROM_245R.Manufacturer = this.a(n7/=2, arrn);
            n7 = arrn[8] & 255;
            n7-=128;
            fT_EEPROM_245R.Product = this.a(n7/=2, arrn);
            n7 = arrn[9] & 255;
            n7-=128;
            fT_EEPROM_245R.SerialNumber = this.a(n7/=2, arrn);
            return fT_EEPROM_245R;
        }
        catch (Exception var3_4) {
            return null;
        }
    }

    int b() {
        int n = this.a(7);
        int n2 = (n & 65280) >> 8;
        n = this.a(8);
        int n3 = (n & 65280) >> 8;
        int n4 = 12 + (n2/=2) + (n3/=2) + 1;
        n = this.a(9);
        int n5 = (n & 65280) >> 8;
        return (63 - n4 - (n5/=2) - 1) * 2;
    }

    int a(byte[] arrby) {
        int n;
        int n2 = 0;
        short s = 0;
        if (arrby.length > this.b()) {
            return 0;
        }
        int[] arrn = new int[80];
        for (n = 0; n < 80; n = (short)(n + 1)) {
            arrn[n] = this.a((short)n);
        }
        s = (short)(63 - this.b() / 2 - 1);
        s = (short)(s & 65535);
        for (n = 0; n < arrby.length; n+=2) {
            n2 = n + 1 < arrby.length ? arrby[n + 1] & 255 : 0;
            n2<<=8;
            short s2 = s;
            s = (short)(s2 + 1);
            arrn[s2] = n2|=arrby[n] & 255;
        }
        n = 0;
        if (arrn[1] != 0 && arrn[2] != 0) {
            boolean bl = false;
            n = d.getLatencyTimer();
            d.setLatencyTimer(119);
            bl = this.a(arrn, 63);
            d.setLatencyTimer((byte)n);
            if (!bl) {
                return 0;
            }
        } else {
            return 0;
        }
        return arrby.length;
    }

    byte[] a(int n) {
        byte by = 0;
        byte by2 = 0;
        int n2 = 0;
        byte[] arrby = new byte[n];
        if (n == 0 || n > this.b()) {
            return null;
        }
        short s = (short)(63 - this.b() / 2 - 1);
        for (int i = 0; i < n; i+=2) {
            short s2 = s;
            s = (short)(s2 + 1);
            n2 = this.a(s2);
            if (i + 1 < arrby.length) {
                arrby[i + 1] = by = (byte)(n2 & 255);
            } else {
                by2 = 0;
            }
            arrby[i] = by2 = (byte)((n2 & 65280) >> 8);
        }
        return arrby;
    }
}

