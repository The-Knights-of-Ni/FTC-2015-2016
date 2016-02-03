/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.hardware.DeviceInterfaceModule
 *  com.qualcomm.robotcore.hardware.TouchSensor
 */
package com.qualcomm.hardware.modernrobotics;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ModernRoboticsDigitalTouchSensor
extends TouchSensor {
    private DeviceInterfaceModule a = null;
    private int b = -1;

    public ModernRoboticsDigitalTouchSensor(DeviceInterfaceModule module, int physicalPort) {
        this.a = module;
        this.b = physicalPort;
    }

    public double getValue() {
        return this.isPressed() ? 1.0 : 0.0;
    }

    public boolean isPressed() {
        return this.a.getDigitalChannelState(this.b);
    }

    public String getDeviceName() {
        return "Modern Robotics Digital Touch Sensor";
    }

    public String getConnectionInfo() {
        return this.a.getConnectionInfo() + "; digital port " + this.b;
    }

    public int getVersion() {
        return 1;
    }

    public void close() {
    }
}

