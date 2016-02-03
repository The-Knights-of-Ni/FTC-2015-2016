/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.app.PendingIntent
 *  android.content.BroadcastReceiver
 *  android.content.Context
 *  android.content.Intent
 *  android.content.IntentFilter
 *  android.hardware.usb.UsbDevice
 *  android.hardware.usb.UsbInterface
 *  android.hardware.usb.UsbManager
 *  android.os.Parcelable
 *  android.util.Log
 */
package com.ftdi.j2xx;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Parcelable;
import android.util.Log;
import com.ftdi.j2xx.FT_Device;
import com.ftdi.j2xx.m;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

public class D2xxManager {
    private static D2xxManager a = null;
    protected static final String ACTION_USB_PERMISSION = "com.ftdi.j2xx";
    public static final byte FT_DATA_BITS_7 = 7;
    public static final byte FT_DATA_BITS_8 = 8;
    public static final byte FT_STOP_BITS_1 = 0;
    public static final byte FT_STOP_BITS_2 = 2;
    public static final byte FT_PARITY_NONE = 0;
    public static final byte FT_PARITY_ODD = 1;
    public static final byte FT_PARITY_EVEN = 2;
    public static final byte FT_PARITY_MARK = 3;
    public static final byte FT_PARITY_SPACE = 4;
    public static final short FT_FLOW_NONE = 0;
    public static final short FT_FLOW_RTS_CTS = 256;
    public static final short FT_FLOW_DTR_DSR = 512;
    public static final short FT_FLOW_XON_XOFF = 1024;
    public static final byte FT_PURGE_RX = 1;
    public static final byte FT_PURGE_TX = 2;
    public static final byte FT_CTS = 16;
    public static final byte FT_DSR = 32;
    public static final byte FT_RI = 64;
    public static final byte FT_DCD = -128;
    public static final byte FT_OE = 2;
    public static final byte FT_PE = 4;
    public static final byte FT_FE = 8;
    public static final byte FT_BI = 16;
    public static final byte FT_EVENT_RXCHAR = 1;
    public static final byte FT_EVENT_MODEM_STATUS = 2;
    public static final byte FT_EVENT_LINE_STATUS = 4;
    public static final byte FT_EVENT_REMOVED = 8;
    public static final byte FT_FLAGS_OPENED = 1;
    public static final byte FT_FLAGS_HI_SPEED = 2;
    public static final int FT_DEVICE_232B = 0;
    public static final int FT_DEVICE_8U232AM = 1;
    public static final int FT_DEVICE_UNKNOWN = 3;
    public static final int FT_DEVICE_2232 = 4;
    public static final int FT_DEVICE_232R = 5;
    public static final int FT_DEVICE_245R = 5;
    public static final int FT_DEVICE_2232H = 6;
    public static final int FT_DEVICE_4232H = 7;
    public static final int FT_DEVICE_232H = 8;
    public static final int FT_DEVICE_X_SERIES = 9;
    public static final int FT_DEVICE_4222_0 = 10;
    public static final int FT_DEVICE_4222_1_2 = 11;
    public static final int FT_DEVICE_4222_3 = 12;
    public static final byte FT_BITMODE_RESET = 0;
    public static final byte FT_BITMODE_ASYNC_BITBANG = 1;
    public static final byte FT_BITMODE_MPSSE = 2;
    public static final byte FT_BITMODE_SYNC_BITBANG = 4;
    public static final byte FT_BITMODE_MCU_HOST = 8;
    public static final byte FT_BITMODE_FAST_SERIAL = 16;
    public static final byte FT_BITMODE_CBUS_BITBANG = 32;
    public static final byte FT_BITMODE_SYNC_FIFO = 64;
    public static final int FTDI_BREAK_OFF = 0;
    public static final int FTDI_BREAK_ON = 16384;
    private static Context b = null;
    private static PendingIntent c = null;
    private static IntentFilter d = null;
    private static List<m> e = new ArrayList<m>(Arrays.asList(new m(1027, 24597), new m(1027, 24596), new m(1027, 24593), new m(1027, 24592), new m(1027, 24577), new m(1027, 24582), new m(1027, 24604), new m(1027, 64193), new m(1027, 64194), new m(1027, 64195), new m(1027, 64196), new m(1027, 64197), new m(1027, 64198), new m(1027, 24594), new m(2220, 4133), new m(5590, 1), new m(1027, 24599)));
    private ArrayList<FT_Device> f;
    private static UsbManager g;
    private BroadcastReceiver h;
    private static BroadcastReceiver i;

    static {
        i = new BroadcastReceiver(){

            public void onReceive(Context context, Intent intent) {
                String string = intent.getAction();
                if ("com.ftdi.j2xx".equals(string)) {
                     var4_4 = this;
                    synchronized (var4_4) {
                        UsbDevice usbDevice = (UsbDevice)intent.getParcelableExtra("device");
                        if (!intent.getBooleanExtra("permission", false)) {
                            Log.d((String)"D2xx::", (String)("permission denied for device " + (Object)usbDevice));
                        }
                    }
                }
            }
        };
    }

    private FT_Device a(UsbDevice usbDevice) {
        FT_Device fT_Device = null;
        ArrayList<FT_Device> arrayList = this.f;
        synchronized (arrayList) {
            int n = this.f.size();
            for (int i = 0; i < n; ++i) {
                FT_Device fT_Device2 = this.f.get(i);
                UsbDevice usbDevice2 = fT_Device2.getUsbDevice();
                if (!usbDevice2.equals((Object)usbDevice)) continue;
                fT_Device = fT_Device2;
                break;
            }
        }
        return fT_Device;
    }

    public boolean isFtDevice(UsbDevice dev) {
        boolean bl = false;
        if (b == null) {
            return bl;
        }
        m m = new m(dev.getVendorId(), dev.getProductId());
        if (e.contains(m)) {
            bl = true;
        }
        Log.v((String)"D2xx::", (String)m.toString());
        return bl;
    }

    private static synchronized boolean a(Context context) {
        boolean bl = false;
        if (context == null) {
            return bl;
        }
        if (b != context) {
            b = context;
            c = PendingIntent.getBroadcast((Context)b.getApplicationContext(), (int)0, (Intent)new Intent("com.ftdi.j2xx"), (int)134217728);
            d = new IntentFilter("com.ftdi.j2xx");
            b.getApplicationContext().registerReceiver(i, d);
        }
        bl = true;
        return bl;
    }

    private boolean b(UsbDevice usbDevice) {
        boolean bl = false;
        if (!g.hasPermission(usbDevice)) {
            g.requestPermission(usbDevice, c);
        }
        if (g.hasPermission(usbDevice)) {
            bl = true;
        }
        return bl;
    }

    private D2xxManager(Context parentContext) throws D2xxException {
        this.h = new BroadcastReceiver(){

            public void onReceive(Context context, Intent intent) {
                String string = intent.getAction();
                UsbDevice usbDevice = null;
                FT_Device fT_Device = null;
                if ("android.hardware.usb.action.USB_DEVICE_DETACHED".equals(string)) {
                    usbDevice = (UsbDevice)intent.getParcelableExtra("device");
                    fT_Device = D2xxManager.this.a(usbDevice);
                    while (fT_Device != null) {
                        fT_Device.close();
                        ArrayList arrayList = D2xxManager.this.f;
                        synchronized (arrayList) {
                            D2xxManager.this.f.remove(fT_Device);
                        }
                        fT_Device = D2xxManager.this.a(usbDevice);
                    }
                } else if ("android.hardware.usb.action.USB_DEVICE_ATTACHED".equals(string)) {
                    usbDevice = (UsbDevice)intent.getParcelableExtra("device");
                    D2xxManager.this.addUsbDevice(usbDevice);
                }
            }
        };
        Log.v((String)"D2xx::", (String)"Start constructor");
        if (parentContext == null) {
            throw new D2xxException("D2xx init failed: Can not find parentContext!");
        }
        D2xxManager.a(parentContext);
        if (!D2xxManager.a()) {
            throw new D2xxException("D2xx init failed: Can not find UsbManager!");
        }
        this.f = new ArrayList();
        IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction("android.hardware.usb.action.USB_DEVICE_ATTACHED");
        intentFilter.addAction("android.hardware.usb.action.USB_DEVICE_DETACHED");
        parentContext.getApplicationContext().registerReceiver(this.h, intentFilter);
        Log.v((String)"D2xx::", (String)"End constructor");
    }

    public static synchronized D2xxManager getInstance(Context parentContext) throws D2xxException {
        if (a == null) {
            a = new D2xxManager(parentContext);
        }
        if (parentContext != null) {
            D2xxManager.a(parentContext);
        }
        return a;
    }

    private static boolean a() {
        if (g == null && b != null) {
            g = (UsbManager)b.getApplicationContext().getSystemService("usb");
        }
        return g != null;
    }

    public boolean setVIDPID(int vendorId, int productId) {
        boolean bl = false;
        if (vendorId != 0 && productId != 0) {
            m m = new m(vendorId, productId);
            if (e.contains(m)) {
                Log.i((String)"D2xx::", (String)("Existing vid:" + vendorId + "  pid:" + productId));
                return true;
            }
            if (!e.add(m)) {
                Log.d((String)"D2xx::", (String)"Failed to add VID/PID combination to list.");
            } else {
                bl = true;
            }
        } else {
            Log.d((String)"D2xx::", (String)"Invalid parameter to setVIDPID");
        }
        return bl;
    }

    public int[][] getVIDPID() {
        int n = e.size();
        int[][] arrn = new int[2][n];
        for (int i = 0; i < n; ++i) {
            m m = e.get(i);
            arrn[0][i] = m.a();
            arrn[1][i] = m.b();
        }
        return arrn;
    }

    private void b() {
        ArrayList<FT_Device> arrayList = this.f;
        synchronized (arrayList) {
            int n = this.f.size();
            for (int i = 0; i < n; ++i) {
                this.f.remove(0);
            }
        }
    }

    public int createDeviceInfoList(Context parentContext) {
        UsbDevice usbDevice;
        HashMap hashMap = g.getDeviceList();
        Iterator iterator = hashMap.values().iterator();
        ArrayList<FT_Device> arrayList = new ArrayList<FT_Device>();
        FT_Device fT_Device = null;
        int n = 0;
        if (parentContext == null) {
            return n;
        }
        D2xxManager.a(parentContext);
        while (iterator.hasNext()) {
            usbDevice = (UsbDevice)iterator.next();
            if (!this.isFtDevice(usbDevice)) continue;
            int n2 = 0;
            n2 = usbDevice.getInterfaceCount();
            for (int i = 0; i < n2; ++i) {
                if (!this.b(usbDevice)) continue;
                ArrayList<FT_Device> arrayList2 = this.f;
                synchronized (arrayList2) {
                    fT_Device = this.a(usbDevice);
                    if (fT_Device == null) {
                        fT_Device = new FT_Device(parentContext, g, usbDevice, usbDevice.getInterface(i));
                    } else {
                        this.f.remove(fT_Device);
                        fT_Device.a(parentContext);
                    }
                    arrayList.add(fT_Device);
                    continue;
                }
            }
        }
        usbDevice = this.f;
        synchronized (usbDevice) {
            this.b();
            this.f = arrayList;
            n = this.f.size();
        }
        return n;
    }

    public synchronized int getDeviceInfoList(int numDevs, FtDeviceInfoListNode[] deviceList) {
        for (int i = 0; i < numDevs; ++i) {
            deviceList[i] = this.f.get((int)i).g;
        }
        return this.f.size();
    }

    public synchronized FtDeviceInfoListNode getDeviceInfoListDetail(int index) {
        if (index > this.f.size() || index < 0) {
            return null;
        }
        return this.f.get((int)index).g;
    }

    public static int getLibraryVersion() {
        return 540016640;
    }

    private boolean a(Context context, FT_Device fT_Device, DriverParameters driverParameters) {
        boolean bl = false;
        if (fT_Device == null) {
            return bl;
        }
        if (context == null) {
            return bl;
        }
        fT_Device.a(context);
        if (driverParameters != null) {
            fT_Device.setDriverParameters(driverParameters);
        }
        if (fT_Device.a(g) && fT_Device.isOpen()) {
            bl = true;
        }
        return bl;
    }

    public synchronized FT_Device openByUsbDevice(Context parentContext, UsbDevice dev, DriverParameters params) {
        FT_Device fT_Device = null;
        if (this.isFtDevice(dev) && !this.a(parentContext, fT_Device = this.a(dev), params)) {
            fT_Device = null;
        }
        return fT_Device;
    }

    public synchronized FT_Device openByUsbDevice(Context parentContext, UsbDevice dev) {
        return this.openByUsbDevice(parentContext, dev, null);
    }

    public synchronized FT_Device openByIndex(Context parentContext, int index, DriverParameters params) {
        FT_Device fT_Device = null;
        if (index < 0) {
            return fT_Device;
        }
        if (parentContext == null) {
            return fT_Device;
        }
        D2xxManager.a(parentContext);
        fT_Device = this.f.get(index);
        if (!this.a(parentContext, fT_Device, params)) {
            fT_Device = null;
        }
        return fT_Device;
    }

    public synchronized FT_Device openByIndex(Context parentContext, int index) {
        return this.openByIndex(parentContext, index, null);
    }

    public synchronized FT_Device openBySerialNumber(Context parentContext, String serialNumber, DriverParameters params) {
        FtDeviceInfoListNode ftDeviceInfoListNode = null;
        FT_Device fT_Device = null;
        if (parentContext == null) {
            return fT_Device;
        }
        D2xxManager.a(parentContext);
        for (int i = 0; i < this.f.size(); ++i) {
            FT_Device fT_Device2 = this.f.get(i);
            if (fT_Device2 == null) continue;
            ftDeviceInfoListNode = fT_Device2.g;
            if (ftDeviceInfoListNode == null) {
                Log.d((String)"D2xx::", (String)"***devInfo cannot be null***");
                continue;
            }
            if (!ftDeviceInfoListNode.serialNumber.equals(serialNumber)) continue;
            fT_Device = fT_Device2;
            break;
        }
        if (!this.a(parentContext, fT_Device, params)) {
            fT_Device = null;
        }
        return fT_Device;
    }

    public synchronized FT_Device openBySerialNumber(Context parentContext, String serialNumber) {
        return this.openBySerialNumber(parentContext, serialNumber, null);
    }

    public synchronized FT_Device openByDescription(Context parentContext, String description, DriverParameters params) {
        FtDeviceInfoListNode ftDeviceInfoListNode = null;
        FT_Device fT_Device = null;
        if (parentContext == null) {
            return fT_Device;
        }
        D2xxManager.a(parentContext);
        for (int i = 0; i < this.f.size(); ++i) {
            FT_Device fT_Device2 = this.f.get(i);
            if (fT_Device2 == null) continue;
            ftDeviceInfoListNode = fT_Device2.g;
            if (ftDeviceInfoListNode == null) {
                Log.d((String)"D2xx::", (String)"***devInfo cannot be null***");
                continue;
            }
            if (!ftDeviceInfoListNode.description.equals(description)) continue;
            fT_Device = fT_Device2;
            break;
        }
        if (!this.a(parentContext, fT_Device, params)) {
            fT_Device = null;
        }
        return fT_Device;
    }

    public synchronized FT_Device openByDescription(Context parentContext, String description) {
        return this.openByDescription(parentContext, description, null);
    }

    public synchronized FT_Device openByLocation(Context parentContext, int location, DriverParameters params) {
        FtDeviceInfoListNode ftDeviceInfoListNode = null;
        FT_Device fT_Device = null;
        if (parentContext == null) {
            return fT_Device;
        }
        D2xxManager.a(parentContext);
        for (int i = 0; i < this.f.size(); ++i) {
            FT_Device fT_Device2 = this.f.get(i);
            if (fT_Device2 == null) continue;
            ftDeviceInfoListNode = fT_Device2.g;
            if (ftDeviceInfoListNode == null) {
                Log.d((String)"D2xx::", (String)"***devInfo cannot be null***");
                continue;
            }
            if (ftDeviceInfoListNode.location != location) continue;
            fT_Device = fT_Device2;
            break;
        }
        if (!this.a(parentContext, fT_Device, params)) {
            fT_Device = null;
        }
        return fT_Device;
    }

    public synchronized FT_Device openByLocation(Context parentContext, int location) {
        return this.openByLocation(parentContext, location, null);
    }

    public int addUsbDevice(UsbDevice dev) {
        int n = 0;
        if (this.isFtDevice(dev)) {
            int n2 = 0;
            n2 = dev.getInterfaceCount();
            for (int i = 0; i < n2; ++i) {
                if (!this.b(dev)) continue;
                ArrayList<FT_Device> arrayList = this.f;
                synchronized (arrayList) {
                    FT_Device fT_Device = this.a(dev);
                    if (fT_Device == null) {
                        fT_Device = new FT_Device(b, g, dev, dev.getInterface(i));
                    } else {
                        fT_Device.a(b);
                        this.f.remove(fT_Device);
                    }
                    this.f.add(fT_Device);
                    ++n;
                    continue;
                }
            }
        }
        return n;
    }

    public static class D2xxException
    extends IOException {
        public D2xxException() {
        }

        public D2xxException(String ftStatusMsg) {
            super(ftStatusMsg);
        }
    }

    public static class DriverParameters {
        private int a = 16384;
        private int b = 16384;
        private int c = 16;
        private int d = 5000;

        public boolean setMaxBufferSize(int size) {
            boolean bl = false;
            if (size >= 64 && size <= 262144) {
                this.a = size;
                bl = true;
            } else {
                Log.e((String)"D2xx::", (String)"***bufferSize Out of correct range***");
            }
            return bl;
        }

        public int getMaxBufferSize() {
            return this.a;
        }

        public boolean setMaxTransferSize(int size) {
            boolean bl = false;
            if (size >= 64 && size <= 262144) {
                this.b = size;
                bl = true;
            } else {
                Log.e((String)"D2xx::", (String)"***maxTransferSize Out of correct range***");
            }
            return bl;
        }

        public int getMaxTransferSize() {
            return this.b;
        }

        public boolean setBufferNumber(int number) {
            boolean bl = false;
            if (number >= 2 && number <= 16) {
                this.c = number;
                bl = true;
            } else {
                Log.e((String)"D2xx::", (String)"***nrBuffers Out of correct range***");
            }
            return bl;
        }

        public int getBufferNumber() {
            return this.c;
        }

        public boolean setReadTimeout(int timeout) {
            this.d = timeout;
            return true;
        }

        public int getReadTimeout() {
            return this.d;
        }
    }

    public static class FtDeviceInfoListNode {
        public int flags;
        public short bcdDevice;
        public int type;
        public byte iSerialNumber;
        public int id;
        public int location;
        public String serialNumber;
        public String description;
        public int handle;
        public int breakOnParam;
        public short modemStatus;
        public short lineStatus;
    }

}

