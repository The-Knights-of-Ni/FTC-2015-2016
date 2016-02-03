/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.hardware.TouchSensorMultiplexer
 *  com.qualcomm.robotcore.util.SerialNumber
 *  com.qualcomm.robotcore.util.TypeConversion
 */
package com.qualcomm.hardware.hitechnic;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbLegacyModule;
import com.qualcomm.robotcore.hardware.TouchSensorMultiplexer;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.TypeConversion;
import java.nio.ByteOrder;

public class HiTechnicNxtTouchSensorMultiplexer
extends TouchSensorMultiplexer {
    int a = 4;
    public static final int MASK_TOUCH_SENSOR_1 = 1;
    public static final int MASK_TOUCH_SENSOR_2 = 2;
    public static final int MASK_TOUCH_SENSOR_3 = 4;
    public static final int MASK_TOUCH_SENSOR_4 = 8;
    public static final int INVALID = -1;
    public static final int[] MASK_MAP = new int[]{-1, 1, 2, 4, 8};
    private final ModernRoboticsUsbLegacyModule b;
    private final int c;

    public HiTechnicNxtTouchSensorMultiplexer(ModernRoboticsUsbLegacyModule legacyModule, int physicalPort) {
        legacyModule.enableAnalogReadMode(physicalPort);
        this.b = legacyModule;
        this.c = physicalPort;
    }

    public String status() {
        return String.format("NXT Touch Sensor Multiplexer, connected via device %s, port %d", this.b.getSerialNumber().toString(), this.c);
    }

    public String getDeviceName() {
        return "NXT Touch Sensor Multiplexer";
    }

    public String getConnectionInfo() {
        return this.b.getConnectionInfo() + "; port " + this.c;
    }

    public int getVersion() {
        return 1;
    }

    public void close() {
    }

    public boolean isTouchSensorPressed(int channel) {
        this.a(channel);
        int n = this.a();
        int n2 = n & MASK_MAP[channel];
        return n2 > 0;
    }

    public int getSwitches() {
        return this.a();
    }

    private int a() {
        byte[] arrby = this.b.readAnalog(3);
        short s = TypeConversion.byteArrayToShort((byte[])arrby, (ByteOrder)ByteOrder.LITTLE_ENDIAN);
        int n = 1023 - s;
        int n2 = 339 * n;
        n2/=1023 - n;
        n2+=5;
        return n2/=10;
    }

    private void a(int n) {
        if (n <= 0 || n > this.a) {
            throw new IllegalArgumentException(String.format("Channel %d is invalid; valid channels are 1..%d", n, this.a));
        }
    }
}

