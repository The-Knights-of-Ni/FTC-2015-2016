/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public abstract class ColorSensor
implements HardwareDevice {
    public abstract int red();

    public abstract int green();

    public abstract int blue();

    public abstract int alpha();

    public abstract int argb();

    public abstract void enableLed(boolean var1);

    public abstract void setI2cAddress(int var1);

    public abstract int getI2cAddress();

    public String toString() {
        return String.format("argb: %d", this.argb());
    }
}

