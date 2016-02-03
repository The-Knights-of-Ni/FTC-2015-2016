/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.net.wifi.WifiManager
 */
package com.qualcomm.robotcore.wifi;

import android.net.wifi.WifiManager;

public class FixWifiDirectSetup {
    public static final int WIFI_TOGGLE_DELAY = 2000;

    public static void fixWifiDirectSetup(WifiManager wifiManager) throws InterruptedException {
        FixWifiDirectSetup.a(false, wifiManager);
        FixWifiDirectSetup.a(true, wifiManager);
    }

    public static void disableWifiDirect(WifiManager wifiManager) throws InterruptedException {
        FixWifiDirectSetup.a(false, wifiManager);
    }

    private static void a(boolean bl, WifiManager wifiManager) throws InterruptedException {
        wifiManager.setWifiEnabled(bl);
        Thread.sleep(2000);
    }
}

