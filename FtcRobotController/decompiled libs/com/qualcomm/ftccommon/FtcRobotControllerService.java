/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.app.Service
 *  android.content.Context
 *  android.content.Intent
 *  android.os.Binder
 *  android.os.Build
 *  android.os.IBinder
 *  com.qualcomm.robotcore.eventloop.EventLoop
 *  com.qualcomm.robotcore.eventloop.EventLoopManager
 *  com.qualcomm.robotcore.eventloop.EventLoopManager$EventLoopMonitor
 *  com.qualcomm.robotcore.exception.RobotCoreException
 *  com.qualcomm.robotcore.factory.RobotFactory
 *  com.qualcomm.robotcore.robot.Robot
 *  com.qualcomm.robotcore.robot.RobotState
 *  com.qualcomm.robotcore.util.ElapsedTime
 *  com.qualcomm.robotcore.util.RobotLog
 *  com.qualcomm.robotcore.util.Util
 *  com.qualcomm.robotcore.wifi.WifiDirectAssistant
 *  com.qualcomm.robotcore.wifi.WifiDirectAssistant$Event
 *  com.qualcomm.robotcore.wifi.WifiDirectAssistant$WifiDirectAssistantCallback
 */
package com.qualcomm.ftccommon;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.os.Binder;
import android.os.Build;
import android.os.IBinder;
import com.qualcomm.ftccommon.ConfigWifiDirectActivity;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.factory.RobotFactory;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Util;
import com.qualcomm.robotcore.wifi.WifiDirectAssistant;
import java.net.InetAddress;

public class FtcRobotControllerService
extends Service
implements WifiDirectAssistant.WifiDirectAssistantCallback {
    private final IBinder a;
    private WifiDirectAssistant b;
    private Robot c;
    private EventLoop d;
    private WifiDirectAssistant.Event e;
    private String f;
    private UpdateUI.Callback g;
    private final a h;
    private final ElapsedTime i;
    private Thread j;

    public FtcRobotControllerService() {
        this.a = new FtcRobotControllerBinder();
        this.e = WifiDirectAssistant.Event.DISCONNECTED;
        this.f = "Robot Status: null";
        this.g = null;
        this.h = new a();
        this.i = new ElapsedTime();
        this.j = null;
    }

    public WifiDirectAssistant getWifiDirectAssistant() {
        return this.b;
    }

    public WifiDirectAssistant.Event getWifiDirectStatus() {
        return this.e;
    }

    public String getRobotStatus() {
        return this.f;
    }

    public IBinder onBind(Intent intent) {
        DbgLog.msg("Starting FTC Controller Service");
        DbgLog.msg("Android device is " + Build.MANUFACTURER + ", " + Build.MODEL);
        this.b = WifiDirectAssistant.getWifiDirectAssistant((Context)this);
        this.b.setCallback((WifiDirectAssistant.WifiDirectAssistantCallback)this);
        this.b.enable();
        if (Build.MODEL.equals("FL7007")) {
            this.b.discoverPeers();
        } else {
            this.b.createGroup();
        }
        return this.a;
    }

    public boolean onUnbind(Intent intent) {
        DbgLog.msg("Stopping FTC Controller Service");
        this.b.disable();
        this.shutdownRobot();
        return false;
    }

    public synchronized void setCallback(UpdateUI.Callback callback) {
        this.g = callback;
    }

    public synchronized void setupRobot(EventLoop eventLoop) {
        if (this.j != null && this.j.isAlive()) {
            DbgLog.msg("FtcRobotControllerService.setupRobot() is currently running, stopping old setup");
            this.j.interrupt();
            while (this.j.isAlive()) {
                Thread.yield();
            }
            DbgLog.msg("Old setup stopped; restarting setup");
        }
        RobotLog.clearGlobalErrorMsg();
        DbgLog.msg("Processing robot setup");
        this.d = eventLoop;
        this.j = new Thread(new b());
        this.j.start();
        while (this.j.getState() == Thread.State.NEW) {
            Thread.yield();
        }
    }

    public synchronized void shutdownRobot() {
        if (this.j != null && this.j.isAlive()) {
            this.j.interrupt();
        }
        if (this.c != null) {
            this.c.shutdown();
        }
        this.c = null;
        this.a("Robot Status: null");
    }

    public void onWifiDirectEvent(WifiDirectAssistant.Event event) {
        switch (event) {
            case CONNECTED_AS_GROUP_OWNER: {
                DbgLog.msg("Wifi Direct - Group Owner");
                this.b.cancelDiscoverPeers();
                if (this.b.isDeviceNameValid()) break;
                RobotLog.e((String)"Wifi-Direct device name contains non-printable characters");
                ConfigWifiDirectActivity.launch(this.getBaseContext(), ConfigWifiDirectActivity.Flag.WIFI_DIRECT_DEVICE_NAME_INVALID);
                break;
            }
            case CONNECTED_AS_PEER: {
                DbgLog.error("Wifi Direct - connected as peer, was expecting Group Owner");
                ConfigWifiDirectActivity.launch(this.getBaseContext(), ConfigWifiDirectActivity.Flag.WIFI_DIRECT_FIX_CONFIG);
                break;
            }
            case CONNECTION_INFO_AVAILABLE: {
                DbgLog.msg("Wifi Direct Passphrase: " + this.b.getPassphrase());
                break;
            }
            case ERROR: {
                DbgLog.error("Wifi Direct Error: " + this.b.getFailureReason());
                break;
            }
        }
        this.a(event);
    }

    private void a(WifiDirectAssistant.Event event) {
        this.e = event;
        if (this.g != null) {
            this.g.wifiDirectUpdate(this.e);
        }
    }

    private void a(String string) {
        this.f = string;
        if (this.g != null) {
            this.g.robotUpdate(string);
        }
    }

    private class b
    implements Runnable {
        private b() {
        }

        @Override
        public void run() {
            Util.logThreadLifeCycle((String)"RobotSetupRunnable.run()", (Runnable)new Runnable(){

                @Override
                public void run() {
                    try {
                        if (FtcRobotControllerService.this.c != null) {
                            FtcRobotControllerService.this.c.shutdown();
                            FtcRobotControllerService.this.c = null;
                        }
                        FtcRobotControllerService.this.a("Robot Status: scanning for USB devices");
                        try {
                            Thread.sleep(5000);
                        }
                        catch (InterruptedException var1_1) {
                            FtcRobotControllerService.this.a("Robot Status: abort due to interrupt");
                            return;
                        }
                        FtcRobotControllerService.this.c = RobotFactory.createRobot();
                        FtcRobotControllerService.this.a("Robot Status: waiting on network");
                        FtcRobotControllerService.this.i.reset();
                        while (!FtcRobotControllerService.this.b.isConnected()) {
                            try {
                                Thread.sleep(1000);
                                if (FtcRobotControllerService.this.i.time() <= 120.0) continue;
                                FtcRobotControllerService.this.a("Robot Status: network timed out");
                                return;
                            }
                            catch (InterruptedException var1_2) {
                                DbgLog.msg("interrupt waiting for network; aborting setup");
                                return;
                            }
                        }
                        FtcRobotControllerService.this.a("Robot Status: starting robot");
                        try {
                            FtcRobotControllerService.b((FtcRobotControllerService)FtcRobotControllerService.this).eventLoopManager.setMonitor((EventLoopManager.EventLoopMonitor)FtcRobotControllerService.this.h);
                            InetAddress inetAddress = FtcRobotControllerService.this.b.getGroupOwnerAddress();
                            FtcRobotControllerService.this.c.start(inetAddress, FtcRobotControllerService.this.d);
                        }
                        catch (RobotCoreException var1_4) {
                            FtcRobotControllerService.this.a("Robot Status: failed to start robot");
                            RobotLog.setGlobalErrorMsg((String)var1_4.getMessage());
                        }
                    }
                    catch (RobotCoreException var1_5) {
                        FtcRobotControllerService.this.a("Robot Status: Unable to create robot!");
                        RobotLog.setGlobalErrorMsg((String)var1_5.getMessage());
                    }
                }
            });
        }

    }

    private class a
    implements EventLoopManager.EventLoopMonitor {
        private a() {
        }

        public void onStateChange(RobotState state) {
            if (FtcRobotControllerService.this.g == null) {
                return;
            }
            switch (state) {
                case INIT: {
                    FtcRobotControllerService.this.g.robotUpdate("Robot Status: init");
                    break;
                }
                case NOT_STARTED: {
                    FtcRobotControllerService.this.g.robotUpdate("Robot Status: not started");
                    break;
                }
                case RUNNING: {
                    FtcRobotControllerService.this.g.robotUpdate("Robot Status: running");
                    break;
                }
                case STOPPED: {
                    FtcRobotControllerService.this.g.robotUpdate("Robot Status: stopped");
                    break;
                }
                case EMERGENCY_STOP: {
                    FtcRobotControllerService.this.g.robotUpdate("Robot Status: EMERGENCY STOP");
                    break;
                }
                case DROPPED_CONNECTION: {
                    FtcRobotControllerService.this.g.robotUpdate("Robot Status: dropped connection");
                }
            }
        }
    }

    public class FtcRobotControllerBinder
    extends Binder {
        public FtcRobotControllerService getService() {
            return FtcRobotControllerService.this;
        }
    }

}

