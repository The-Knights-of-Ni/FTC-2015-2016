/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.content.Context
 *  com.qualcomm.analytics.Analytics
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.hardware.DeviceManager
 *  com.qualcomm.robotcore.hardware.DeviceManager$DeviceType
 *  com.qualcomm.robotcore.hardware.HardwareMap
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbDevice$Channel
 *  com.qualcomm.robotcore.hardware.usb.RobotUsbManager
 *  com.qualcomm.robotcore.util.SerialNumber
 */
package com.qualcomm.modernrobotics;

import android.content.Context;
import com.qualcomm.analytics.Analytics;
import com.qualcomm.modernrobotics.a;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.hardware.usb.RobotUsbManager;
import com.qualcomm.robotcore.util.SerialNumber;
import java.io.PrintStream;

public class ModernRoboticsUsbUtil {
    public static final int MFG_CODE_MODERN_ROBOTICS = 77;
    public static final int DEVICE_ID_DC_MOTOR_CONTROLLER = 77;
    public static final int DEVICE_ID_SERVO_CONTROLLER = 83;
    public static final int DEVICE_ID_LEGACY_MODULE = 73;
    public static final int DEVICE_ID_DEVICE_INTERFACE_MODULE = 65;
    private static Analytics a;

    public static void init(Context context, HardwareMap map) {
        if (a == null) {
            a = new Analytics(context, "update_rc", map);
        }
    }

    public static RobotUsbDevice openUsbDevice(RobotUsbManager usbManager, SerialNumber serialNumber) throws RobotCoreException {
        return ModernRoboticsUsbUtil.a(usbManager, serialNumber);
    }

    private static RobotUsbDevice a(RobotUsbManager robotUsbManager, SerialNumber serialNumber) throws RobotCoreException {
        String string = "";
        boolean bl = false;
        int n = robotUsbManager.scanForDevices();
        for (int i = 0; i < n; ++i) {
            if (!serialNumber.equals((Object)robotUsbManager.getDeviceSerialNumberByIndex(i))) continue;
            bl = true;
            string = robotUsbManager.getDeviceDescriptionByIndex(i) + " [" + serialNumber.getSerialNumber() + "]";
            break;
        }
        if (!bl) {
            ModernRoboticsUsbUtil.a("unable to find USB device with serial number " + serialNumber.toString());
        }
        RobotUsbDevice robotUsbDevice = null;
        try {
            robotUsbDevice = robotUsbManager.openBySerialNumber(serialNumber);
        }
        catch (RobotCoreException var6_7) {
            ModernRoboticsUsbUtil.a("Unable to open USB device " + (Object)serialNumber + " - " + string + ": " + var6_7.getMessage());
        }
        try {
            robotUsbDevice.setBaudRate(250000);
            robotUsbDevice.setDataCharacteristics(8, 0, 0);
            robotUsbDevice.setLatencyTimer(2);
        }
        catch (RobotCoreException var6_8) {
            robotUsbDevice.close();
            ModernRoboticsUsbUtil.a("Unable to open USB device " + (Object)serialNumber + " - " + string + ": " + var6_8.getMessage());
        }
        try {
            Thread.sleep(400);
        }
        catch (InterruptedException var6_9) {
            // empty catch block
        }
        return robotUsbDevice;
    }

    public static byte[] getUsbDeviceHeader(RobotUsbDevice dev) throws RobotCoreException {
        return ModernRoboticsUsbUtil.a(dev);
    }

    private static byte[] a(RobotUsbDevice robotUsbDevice) throws RobotCoreException {
        int n = 3;
        byte[] arrby = new byte[5];
        byte[] arrby2 = new byte[3];
        byte[] arrby3 = new byte[]{85, -86, -128, 0, 3};
        try {
            robotUsbDevice.purge(RobotUsbDevice.Channel.RX);
            robotUsbDevice.write(arrby3);
            robotUsbDevice.read(arrby);
        }
        catch (RobotCoreException var5_5) {
            ModernRoboticsUsbUtil.a("error reading Modern Robotics USB device headers");
        }
        if (!a.a(arrby, 3)) {
            return arrby2;
        }
        robotUsbDevice.read(arrby2);
        return arrby2;
    }

    public static DeviceManager.DeviceType getDeviceType(byte[] deviceHeader) {
        return ModernRoboticsUsbUtil.a(deviceHeader);
    }

    private static DeviceManager.DeviceType a(byte[] arrby) {
        if (arrby[1] != 77) {
            return DeviceManager.DeviceType.FTDI_USB_UNKNOWN_DEVICE;
        }
        switch (arrby[2]) {
            case 77: {
                return DeviceManager.DeviceType.MODERN_ROBOTICS_USB_DC_MOTOR_CONTROLLER;
            }
            case 83: {
                return DeviceManager.DeviceType.MODERN_ROBOTICS_USB_SERVO_CONTROLLER;
            }
            case 73: {
                return DeviceManager.DeviceType.MODERN_ROBOTICS_USB_LEGACY_MODULE;
            }
            case 65: {
                return DeviceManager.DeviceType.MODERN_ROBOTICS_USB_DEVICE_INTERFACE_MODULE;
            }
        }
        return DeviceManager.DeviceType.MODERN_ROBOTICS_USB_UNKNOWN_DEVICE;
    }

    private static void a(String string) throws RobotCoreException {
        System.err.println(string);
        throw new RobotCoreException(string);
    }
}

