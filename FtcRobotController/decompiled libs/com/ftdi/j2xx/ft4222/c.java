/*
 * Decompiled with CFR 0_101.
 */
package com.ftdi.j2xx.ft4222;

class c {
    byte a;
    byte b;
    byte[] c;

    public c(char[] arrc) {
        this.c = arrc[0] < 'B' ? new byte[3] : new byte[1];
    }
}

