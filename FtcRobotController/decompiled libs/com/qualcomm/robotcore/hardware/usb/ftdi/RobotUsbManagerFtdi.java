/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.content.Context
 */
package com.qualcomm.robotcore.hardware.usb.ftdi;

import android.content.Context;
import com.ftdi.j2xx.D2xxManager;
import com.ftdi.j2xx.FT_Device;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.hardware.usb.RobotUsbManager;
import com.qualcomm.robotcore.hardware.usb.ftdi.RobotUsbDeviceFtdi;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;

public class RobotUsbManagerFtdi
implements RobotUsbManager {
    private Context a;
    private D2xxManager b;

    public RobotUsbManagerFtdi(Context context) {
        this.a = context;
        try {
            this.b = D2xxManager.getInstance(context);
        }
        catch (D2xxManager.D2xxException var2_2) {
            RobotLog.e("Unable to create D2xxManager; cannot open USB devices");
        }
    }

    @Override
    public int scanForDevices() throws RobotCoreException {
        return this.b.createDeviceInfoList(this.a);
    }

    @Override
    public SerialNumber getDeviceSerialNumberByIndex(int index) throws RobotCoreException {
        return new SerialNumber(this.b.getDeviceInfoListDetail((int)index).serialNumber);
    }

    @Override
    public String getDeviceDescriptionByIndex(int index) throws RobotCoreException {
        return this.b.getDeviceInfoListDetail((int)index).description;
    }

    @Override
    public RobotUsbDevice openBySerialNumber(SerialNumber serialNumber) throws RobotCoreException {
        FT_Device fT_Device = this.b.openBySerialNumber(this.a, serialNumber.toString());
        if (fT_Device == null) {
            throw new RobotCoreException("FTDI driver failed to open USB device with serial number " + serialNumber + " (returned null device)");
        }
        return new RobotUsbDeviceFtdi(fT_Device);
    }
}

