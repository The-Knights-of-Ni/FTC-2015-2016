/*
 * Decompiled with CFR 0_101.
 */
package com.ftdi.j2xx.interfaces;

public interface I2cMaster {
    public int init(int var1);

    public int reset();

    public int read(int var1, byte[] var2, int var3, int[] var4);

    public int write(int var1, byte[] var2, int var3, int[] var4);
}

