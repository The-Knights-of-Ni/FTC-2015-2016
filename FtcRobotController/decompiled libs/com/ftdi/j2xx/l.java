/*
 * Decompiled with CFR 0_101.
 */
package com.ftdi.j2xx;

import com.ftdi.j2xx.FT_Device;
import com.ftdi.j2xx.FT_EEPROM;
import com.ftdi.j2xx.FT_EEPROM_X_Series;
import com.ftdi.j2xx.k;

class l
extends k {
    private static FT_Device d;

    l(FT_Device fT_Device) {
        super(fT_Device);
        d = fT_Device;
        this.b = 128;
        this.a = 1;
    }

    short a(FT_EEPROM fT_EEPROM) {
        short s;
        int n;
        byte by;
        int n2;
        int n3;
        int[] arrn;
        block46 : {
            arrn = new int[this.b];
            int n4 = 0;
            if (fT_EEPROM.getClass() != FT_EEPROM_X_Series.class) {
                return 1;
            }
            FT_EEPROM_X_Series fT_EEPROM_X_Series = (FT_EEPROM_X_Series)fT_EEPROM;
            do {
                arrn[n4] = this.a((short)n4);
            } while ((n4 = (int)(n4 + 1)) < this.b);
            try {
                arrn[0] = 0;
                if (fT_EEPROM_X_Series.BCDEnable) {
                    int[] arrn2 = arrn;
                    arrn2[0] = arrn2[0] | 1;
                }
                if (fT_EEPROM_X_Series.BCDForceCBusPWREN) {
                    int[] arrn3 = arrn;
                    arrn3[0] = arrn3[0] | 2;
                }
                if (fT_EEPROM_X_Series.BCDDisableSleep) {
                    int[] arrn4 = arrn;
                    arrn4[0] = arrn4[0] | 4;
                }
                if (fT_EEPROM_X_Series.RS485EchoSuppress) {
                    int[] arrn5 = arrn;
                    arrn5[0] = arrn5[0] | 8;
                }
                if (fT_EEPROM_X_Series.A_LoadVCP) {
                    int[] arrn6 = arrn;
                    arrn6[0] = arrn6[0] | 128;
                }
                if (!fT_EEPROM_X_Series.PowerSaveEnable) break block46;
                by = 0;
                if (fT_EEPROM_X_Series.CBus0 == 17) {
                    by = 1;
                }
                if (fT_EEPROM_X_Series.CBus1 == 17) {
                    by = 1;
                }
                if (fT_EEPROM_X_Series.CBus2 == 17) {
                    by = 1;
                }
                if (fT_EEPROM_X_Series.CBus3 == 17) {
                    by = 1;
                }
                if (fT_EEPROM_X_Series.CBus4 == 17) {
                    by = 1;
                }
                if (fT_EEPROM_X_Series.CBus5 == 17) {
                    by = 1;
                }
                if (fT_EEPROM_X_Series.CBus6 == 17) {
                    by = 1;
                }
                if (by != 0) {
                    int[] arrn7 = arrn;
                    arrn7[0] = arrn7[0] | 64;
                    break block46;
                }
                return 1;
            }
            catch (Exception var5_6) {
                var5_6.printStackTrace();
                return 0;
            }
        }
        arrn[1] = fT_EEPROM_X_Series.VendorId;
        arrn[2] = fT_EEPROM_X_Series.ProductId;
        arrn[3] = 4096;
        arrn[4] = this.a((Object)fT_EEPROM);
        arrn[5] = this.b(fT_EEPROM);
        if (fT_EEPROM_X_Series.FT1248ClockPolarity) {
            int[] arrn8 = arrn;
            arrn8[5] = arrn8[5] | 16;
        }
        if (fT_EEPROM_X_Series.FT1248LSB) {
            int[] arrn9 = arrn;
            arrn9[5] = arrn9[5] | 32;
        }
        if (fT_EEPROM_X_Series.FT1248FlowControl) {
            int[] arrn10 = arrn;
            arrn10[5] = arrn10[5] | 64;
        }
        if (fT_EEPROM_X_Series.I2CDisableSchmitt) {
            int[] arrn11 = arrn;
            arrn11[5] = arrn11[5] | 128;
        }
        if (fT_EEPROM_X_Series.InvertTXD) {
            int[] arrn12 = arrn;
            arrn12[5] = arrn12[5] | 256;
        }
        if (fT_EEPROM_X_Series.InvertRXD) {
            int[] arrn13 = arrn;
            arrn13[5] = arrn13[5] | 512;
        }
        if (fT_EEPROM_X_Series.InvertRTS) {
            int[] arrn14 = arrn;
            arrn14[5] = arrn14[5] | 1024;
        }
        if (fT_EEPROM_X_Series.InvertCTS) {
            int[] arrn15 = arrn;
            arrn15[5] = arrn15[5] | 2048;
        }
        if (fT_EEPROM_X_Series.InvertDTR) {
            int[] arrn16 = arrn;
            arrn16[5] = arrn16[5] | 4096;
        }
        if (fT_EEPROM_X_Series.InvertDSR) {
            int[] arrn17 = arrn;
            arrn17[5] = arrn17[5] | 8192;
        }
        if (fT_EEPROM_X_Series.InvertDCD) {
            int[] arrn18 = arrn;
            arrn18[5] = arrn18[5] | 16384;
        }
        if (fT_EEPROM_X_Series.InvertRI) {
            int[] arrn19 = arrn;
            arrn19[5] = arrn19[5] | 32768;
        }
        arrn[6] = 0;
        by = fT_EEPROM_X_Series.AD_DriveCurrent;
        if (by == -1) {
            by = 0;
        }
        int[] arrn20 = arrn;
        arrn20[6] = arrn20[6] | by;
        if (fT_EEPROM_X_Series.AD_SlowSlew) {
            int[] arrn21 = arrn;
            arrn21[6] = arrn21[6] | 4;
        }
        if (fT_EEPROM_X_Series.AD_SchmittInput) {
            int[] arrn22 = arrn;
            arrn22[6] = arrn22[6] | 8;
        }
        if ((s = fT_EEPROM_X_Series.AC_DriveCurrent) == -1) {
            s = 0;
        }
        s = (short)(s << 4);
        int[] arrn23 = arrn;
        arrn23[6] = arrn23[6] | s;
        if (fT_EEPROM_X_Series.AC_SlowSlew) {
            int[] arrn24 = arrn;
            arrn24[6] = arrn24[6] | 64;
        }
        if (fT_EEPROM_X_Series.AC_SchmittInput) {
            int[] arrn25 = arrn;
            arrn25[6] = arrn25[6] | 128;
        }
        int n5 = 80;
        n5 = this.a(fT_EEPROM_X_Series.Manufacturer, arrn, n5, 7, false);
        n5 = this.a(fT_EEPROM_X_Series.Product, arrn, n5, 8, false);
        if (fT_EEPROM_X_Series.SerNumEnable) {
            n5 = this.a(fT_EEPROM_X_Series.SerialNumber, arrn, n5, 9, false);
        }
        arrn[10] = fT_EEPROM_X_Series.I2CSlaveAddress;
        arrn[11] = fT_EEPROM_X_Series.I2CDeviceID & 65535;
        arrn[12] = fT_EEPROM_X_Series.I2CDeviceID >> 16;
        byte by2 = fT_EEPROM_X_Series.CBus0;
        if (by2 == -1) {
            by2 = 0;
        }
        if ((n = fT_EEPROM_X_Series.CBus1) == -1) {
            n = 0;
        }
        arrn[13] = (short)(by2 | (n<<=8));
        byte by3 = fT_EEPROM_X_Series.CBus2;
        if (by3 == -1) {
            by3 = 0;
        }
        if ((n2 = fT_EEPROM_X_Series.CBus3) == -1) {
            n2 = 0;
        }
        arrn[14] = (short)(by3 | (n2<<=8));
        byte by4 = fT_EEPROM_X_Series.CBus4;
        if (by4 == -1) {
            by4 = 0;
        }
        if ((n3 = fT_EEPROM_X_Series.CBus5) == -1) {
            n3 = 0;
        }
        arrn[15] = (short)(by4 | (n3<<=8));
        short s2 = fT_EEPROM_X_Series.CBus6;
        if (s2 == -1) {
            s2 = 0;
        }
        arrn[16] = s2;
        if (arrn[1] != 0 && arrn[2] != 0) {
            boolean bl = false;
            bl = this.b(arrn, this.b - 1);
            if (bl) {
                return 0;
            }
            return 1;
        }
        return 2;
    }

    boolean b(int[] arrn, int n) {
        int n2 = n;
        int n3 = 43690;
        int n4 = 0;
        int n5 = 0;
        int n6 = 0;
        int n7 = 0;
        int n8 = 0;
        do {
            n8 = arrn[n5];
            this.a((short)n5, (short)(n8&=65535));
            n4 = n8 ^ n3;
            n6 = (n4&=65535) << 1;
            n7 = (n4 & 32768) > 0 ? 1 : 0;
            n3 = (n6&=65535) | n7;
            n3&=65535;
            if (++n5 != 18) continue;
            n5 = 64;
        } while (n5 != n2);
        this.a((short)n2, (short)n3);
        return true;
    }

    FT_EEPROM a() {
        FT_EEPROM_X_Series fT_EEPROM_X_Series = new FT_EEPROM_X_Series();
        int[] arrn = new int[this.b];
        try {
            short s;
            for (s = 0; s < this.b; s = (short)(s + 1)) {
                arrn[s] = this.a(s);
            }
            fT_EEPROM_X_Series.BCDEnable = (arrn[0] & 1) > 0;
            fT_EEPROM_X_Series.BCDForceCBusPWREN = (arrn[0] & 2) > 0;
            fT_EEPROM_X_Series.BCDDisableSleep = (arrn[0] & 4) > 0;
            fT_EEPROM_X_Series.RS485EchoSuppress = (arrn[0] & 8) > 0;
            fT_EEPROM_X_Series.PowerSaveEnable = (arrn[0] & 64) > 0;
            if ((arrn[0] & 128) > 0) {
                fT_EEPROM_X_Series.A_LoadVCP = true;
                fT_EEPROM_X_Series.A_LoadD2XX = false;
            } else {
                fT_EEPROM_X_Series.A_LoadVCP = false;
                fT_EEPROM_X_Series.A_LoadD2XX = true;
            }
            fT_EEPROM_X_Series.VendorId = (short)arrn[1];
            fT_EEPROM_X_Series.ProductId = (short)arrn[2];
            this.a(fT_EEPROM_X_Series, arrn[4]);
            this.a((Object)fT_EEPROM_X_Series, arrn[5]);
            fT_EEPROM_X_Series.FT1248ClockPolarity = (arrn[5] & 16) > 0;
            fT_EEPROM_X_Series.FT1248LSB = (arrn[5] & 32) > 0;
            fT_EEPROM_X_Series.FT1248FlowControl = (arrn[5] & 64) > 0;
            fT_EEPROM_X_Series.I2CDisableSchmitt = (arrn[5] & 128) > 0;
            fT_EEPROM_X_Series.InvertTXD = (arrn[5] & 256) == 256;
            fT_EEPROM_X_Series.InvertRXD = (arrn[5] & 512) == 512;
            fT_EEPROM_X_Series.InvertRTS = (arrn[5] & 1024) == 1024;
            fT_EEPROM_X_Series.InvertCTS = (arrn[5] & 2048) == 2048;
            fT_EEPROM_X_Series.InvertDTR = (arrn[5] & 4096) == 4096;
            fT_EEPROM_X_Series.InvertDSR = (arrn[5] & 8192) == 8192;
            fT_EEPROM_X_Series.InvertDCD = (arrn[5] & 16384) == 16384;
            fT_EEPROM_X_Series.InvertRI = (arrn[5] & 32768) == 32768;
            s = (short)(arrn[6] & 3);
            switch (s) {
                case 0: {
                    fT_EEPROM_X_Series.AD_DriveCurrent = 0;
                    break;
                }
                case 1: {
                    fT_EEPROM_X_Series.AD_DriveCurrent = 1;
                    break;
                }
                case 2: {
                    fT_EEPROM_X_Series.AD_DriveCurrent = 2;
                    break;
                }
                case 3: {
                    fT_EEPROM_X_Series.AD_DriveCurrent = 3;
                    break;
                }
            }
            short s2 = (short)(arrn[6] & 4);
            fT_EEPROM_X_Series.AD_SlowSlew = s2 == 4;
            short s3 = (short)(arrn[6] & 8);
            fT_EEPROM_X_Series.AD_SchmittInput = s3 == 8;
            short s4 = (short)((arrn[6] & 48) >> 4);
            switch (s4) {
                case 0: {
                    fT_EEPROM_X_Series.AC_DriveCurrent = 0;
                    break;
                }
                case 1: {
                    fT_EEPROM_X_Series.AC_DriveCurrent = 1;
                    break;
                }
                case 2: {
                    fT_EEPROM_X_Series.AC_DriveCurrent = 2;
                    break;
                }
                case 3: {
                    fT_EEPROM_X_Series.AC_DriveCurrent = 3;
                    break;
                }
            }
            short s5 = (short)(arrn[6] & 64);
            fT_EEPROM_X_Series.AC_SlowSlew = s5 == 64;
            short s6 = (short)(arrn[6] & 128);
            fT_EEPROM_X_Series.AC_SchmittInput = s6 == 128;
            fT_EEPROM_X_Series.I2CSlaveAddress = arrn[10];
            fT_EEPROM_X_Series.I2CDeviceID = arrn[11];
            fT_EEPROM_X_Series.I2CDeviceID|=(arrn[12] & 255) << 16;
            fT_EEPROM_X_Series.CBus0 = (byte)(arrn[13] & 255);
            fT_EEPROM_X_Series.CBus1 = (byte)(arrn[13] >> 8 & 255);
            fT_EEPROM_X_Series.CBus2 = (byte)(arrn[14] & 255);
            fT_EEPROM_X_Series.CBus3 = (byte)(arrn[14] >> 8 & 255);
            fT_EEPROM_X_Series.CBus4 = (byte)(arrn[15] & 255);
            fT_EEPROM_X_Series.CBus5 = (byte)(arrn[15] >> 8 & 255);
            fT_EEPROM_X_Series.CBus6 = (byte)(arrn[16] & 255);
            this.a = (short)(arrn[73] >> 8);
            int n = arrn[7] & 255;
            fT_EEPROM_X_Series.Manufacturer = this.a(n/=2, arrn);
            n = arrn[8] & 255;
            fT_EEPROM_X_Series.Product = this.a(n/=2, arrn);
            n = arrn[9] & 255;
            fT_EEPROM_X_Series.SerialNumber = this.a(n/=2, arrn);
            return fT_EEPROM_X_Series;
        }
        catch (Exception var3_4) {
            return null;
        }
    }

    int b() {
        int n = this.a(9);
        int n2 = n & 255;
        n2/=2;
        int n3 = (n & 65280) >> 8;
        n2+=n3 / 2;
        return (this.b - 1 - 1 - ++n2) * 2;
    }

    int a(byte[] arrby) {
        int n;
        int n2 = 0;
        short s = 0;
        if (arrby.length > this.b()) {
            return 0;
        }
        int[] arrn = new int[this.b];
        for (n = 0; n < this.b; n = (short)(n + 1)) {
            arrn[n] = this.a((short)n);
        }
        s = (short)(this.b - this.b() / 2 - 1 - 1);
        for (n = 0; n < arrby.length; n+=2) {
            n2 = n + 1 < arrby.length ? arrby[n + 1] & 255 : 0;
            n2<<=8;
            short s2 = s;
            s = (short)(s2 + 1);
            arrn[s2] = n2|=arrby[n] & 255;
        }
        if (arrn[1] != 0 && arrn[2] != 0) {
            n = 0;
            n = (int)this.b(arrn, this.b - 1) ? 1 : 0;
            if (n == 0) {
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
        short s = (short)(this.b - this.b() / 2 - 1 - 1);
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

