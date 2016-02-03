/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.hardware.OpticalDistanceSensor
 *  com.qualcomm.robotcore.util.SerialNumber
 */
package com.qualcomm.hardware.modernrobotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.SerialNumber;

public class ModernRoboticsAnalogOpticalDistanceSensor
extends OpticalDistanceSensor {
    private final ModernRoboticsUsbDeviceInterfaceModule a;
    private final int b;

    public ModernRoboticsAnalogOpticalDistanceSensor(ModernRoboticsUsbDeviceInterfaceModule deviceInterfaceModule, int physicalPort) {
        this.a = deviceInterfaceModule;
        this.b = physicalPort;
    }

    public double getLightDetected() {
        return (double)this.a.getAnalogInputValue(this.b) / 1023.0;
    }

    public int getLightDetectedRaw() {
        return this.a.getAnalogInputValue(this.b);
    }

    public void enableLed(boolean enable) {
    }

    public String status() {
        return String.format("Optical Distance Sensor, connected via device %s, port %d", this.a.getSerialNumber().toString(), this.b);
    }

    public String getDeviceName() {
        return "Modern Robotics Analog Optical Distance Sensor";
    }

    public String getConnectionInfo() {
        return this.a.getConnectionInfo() + "; analog port " + this.b;
    }

    public int getVersion() {
        return 0;
    }

    public void close() {
    }
}

