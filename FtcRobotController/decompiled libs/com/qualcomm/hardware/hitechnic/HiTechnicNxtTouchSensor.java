/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.hardware.TouchSensor
 *  com.qualcomm.robotcore.util.SerialNumber
 *  com.qualcomm.robotcore.util.TypeConversion
 */
package com.qualcomm.hardware.hitechnic;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbLegacyModule;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.TypeConversion;
import java.nio.ByteOrder;

public class HiTechnicNxtTouchSensor
extends TouchSensor {
    private final ModernRoboticsUsbLegacyModule a;
    private final int b;

    public HiTechnicNxtTouchSensor(ModernRoboticsUsbLegacyModule legacyModule, int physicalPort) {
        legacyModule.enableAnalogReadMode(physicalPort);
        this.a = legacyModule;
        this.b = physicalPort;
    }

    public String status() {
        return String.format("NXT Touch Sensor, connected via device %s, port %d", this.a.getSerialNumber().toString(), this.b);
    }

    public double getValue() {
        double d = TypeConversion.byteArrayToShort((byte[])this.a.readAnalog(this.b), (ByteOrder)ByteOrder.LITTLE_ENDIAN);
        d = d > 675.0 ? 0.0 : 1.0;
        return d;
    }

    public boolean isPressed() {
        return this.getValue() > 0.0;
    }

    public String getDeviceName() {
        return "NXT Touch Sensor";
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

