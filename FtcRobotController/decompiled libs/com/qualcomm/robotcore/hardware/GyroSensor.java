/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public abstract class GyroSensor
implements HardwareDevice {
    public abstract void calibrate();

    public abstract boolean isCalibrating();

    public abstract int getHeading();

    public abstract double getRotation();

    public abstract int rawX();

    public abstract int rawY();

    public abstract int rawZ();

    public abstract void resetZAxisIntegrator();

    public abstract String status();

    public String toString() {
        return String.format("Gyro: %3.1f", this.getRotation());
    }

    protected void notSupported() {
        throw new UnsupportedOperationException("This method is not supported for " + this.getDeviceName());
    }
}

