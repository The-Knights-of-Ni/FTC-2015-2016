/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.hardware.LightSensor
 *  com.qualcomm.robotcore.util.Range
 *  com.qualcomm.robotcore.util.SerialNumber
 *  com.qualcomm.robotcore.util.TypeConversion
 */
package com.qualcomm.hardware.hitechnic;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbLegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.TypeConversion;

public class HiTechnicNxtLightSensor
extends LightSensor {
    public static final byte LED_DIGITAL_LINE_NUMBER = 0;
    private final ModernRoboticsUsbLegacyModule a;
    private final int b;

    public HiTechnicNxtLightSensor(ModernRoboticsUsbLegacyModule legacyModule, int physicalPort) {
        legacyModule.enableAnalogReadMode(physicalPort);
        this.a = legacyModule;
        this.b = physicalPort;
    }

    public double getLightDetected() {
        byte[] arrby = this.a.readAnalog(this.b);
        return Range.scale((double)arrby[0], (double)-128.0, (double)127.0, (double)0.0, (double)1.0);
    }

    public int getLightDetectedRaw() {
        byte[] arrby = this.a.readAnalog(this.b);
        return TypeConversion.unsignedByteToInt((byte)arrby[0]);
    }

    public void enableLed(boolean enable) {
        this.a.setDigitalLine(this.b, 0, enable);
    }

    public String status() {
        return String.format("NXT Light Sensor, connected via device %s, port %d", this.a.getSerialNumber().toString(), this.b);
    }

    public String getDeviceName() {
        return "NXT Light Sensor";
    }

    public String getConnectionInfo() {
        return this.a.getConnectionInfo() + "; port " + this.b;
    }

    public int getVersion() {
        return 1;
    }

    public void close() {
    }
}

