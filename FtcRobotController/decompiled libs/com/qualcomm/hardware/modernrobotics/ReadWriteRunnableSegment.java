/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.hardware.modernrobotics;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class ReadWriteRunnableSegment {
    private int c;
    final Lock a;
    private final byte[] d;
    final Lock b;
    private final byte[] e;

    public ReadWriteRunnableSegment(int address, int size) {
        this.c = address;
        this.a = new ReentrantLock();
        this.d = new byte[size];
        this.b = new ReentrantLock();
        this.e = new byte[size];
    }

    public int getAddress() {
        return this.c;
    }

    public void setAddress(int address) {
        this.c = address;
    }

    public Lock getReadLock() {
        return this.a;
    }

    public byte[] getReadBuffer() {
        return this.d;
    }

    public Lock getWriteLock() {
        return this.b;
    }

    public byte[] getWriteBuffer() {
        return this.e;
    }

    public String toString() {
        return String.format("Segment - address:%d read:%d write:%d", this.c, this.d.length, this.e.length);
    }
}

