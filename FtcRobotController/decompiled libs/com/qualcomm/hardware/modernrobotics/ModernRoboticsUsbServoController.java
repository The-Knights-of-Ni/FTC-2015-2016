/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  com.qualcomm.robotcore.eventloop.EventLoopManager
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.ServoController
 *  com.qualcomm.robotcore.hardware.ServoController$PwmStatus
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
 *  com.qualcomm.robotcore.util.Range
 *  com.qualcomm.robotcore.util.SerialNumber
 *  com.qualcomm.robotcore.util.TypeConversion
 */
package com.qualcomm.hardware.modernrobotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDevice;
import com.qualcomm.hardware.modernrobotics.ReadWriteRunnable;
import com.qualcomm.hardware.modernrobotics.ReadWriteRunnableBlocking;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.TypeConversion;

public class ModernRoboticsUsbServoController
extends ModernRoboticsUsbDevice
implements ServoController {
    public static final boolean DEBUG_LOGGING = false;
    public static final int MONITOR_LENGTH = 9;
    public static final int MAX_SERVOS = 6;
    public static final int SERVO_POSITION_MAX = 255;
    public static final byte PWM_ENABLE = 0;
    public static final byte PWM_ENABLE_WITHOUT_TIMEOUT = -86;
    public static final byte PWM_DISABLE = -1;
    public static final byte START_ADDRESS = 64;
    public static final int ADDRESS_CHANNEL1 = 66;
    public static final int ADDRESS_CHANNEL2 = 67;
    public static final int ADDRESS_CHANNEL3 = 68;
    public static final int ADDRESS_CHANNEL4 = 69;
    public static final int ADDRESS_CHANNEL5 = 70;
    public static final int ADDRESS_CHANNEL6 = 71;
    public static final int ADDRESS_PWM = 72;
    public static final int ADDRESS_UNUSED = -1;
    public static final byte[] ADDRESS_CHANNEL_MAP = new byte[]{-1, 66, 67, 68, 69, 70, 71};

    public ModernRoboticsUsbServoController(SerialNumber serialNumber, RobotUsbDevice device, EventLoopManager manager) throws RobotCoreException, InterruptedException {
        super(serialNumber, manager, new ReadWriteRunnableBlocking(serialNumber, device, 9, 64, false));
        this.pwmDisable();
    }

    @Override
    public String getDeviceName() {
        return "Modern Robotics USB Servo Controller";
    }

    public String getConnectionInfo() {
        return "USB " + (Object)this.getSerialNumber();
    }

    @Override
    public void close() {
        this.pwmDisable();
        super.close();
    }

    public void pwmEnable() {
        this.write(72, 0);
    }

    public void pwmDisable() {
        this.write(72, -1);
    }

    public ServoController.PwmStatus getPwmStatus() {
        byte[] arrby = this.read(72, 1);
        if (arrby[0] == -1) {
            return ServoController.PwmStatus.DISABLED;
        }
        return ServoController.PwmStatus.ENABLED;
    }

    public void setServoPosition(int channel, double position) {
        this.a(channel);
        Range.throwIfRangeIsInvalid((double)position, (double)0.0, (double)1.0);
        this.write((int)ADDRESS_CHANNEL_MAP[channel], position * 255.0);
        this.pwmEnable();
    }

    public double getServoPosition(int channel) {
        this.a(channel);
        byte[] arrby = this.read(ADDRESS_CHANNEL_MAP[channel], 1);
        return TypeConversion.unsignedByteToDouble((byte)arrby[0]) / 255.0;
    }

    private void a(int n) {
        if (n < 1 || n > ADDRESS_CHANNEL_MAP.length) {
            throw new IllegalArgumentException(String.format("Channel %d is invalid; valid channels are 1..%d", n, 6));
        }
    }
}

