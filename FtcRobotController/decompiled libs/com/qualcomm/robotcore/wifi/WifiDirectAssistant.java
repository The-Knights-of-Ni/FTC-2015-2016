/*
 * Decompiled with CFR 0_101.
 * 
 * Could not load the following classes:
 *  android.content.BroadcastReceiver
 *  android.content.Context
 *  android.content.Intent
 *  android.content.IntentFilter
 *  android.net.NetworkInfo
 *  android.net.wifi.WpsInfo
 *  android.net.wifi.p2p.WifiP2pConfig
 *  android.net.wifi.p2p.WifiP2pDevice
 *  android.net.wifi.p2p.WifiP2pDeviceList
 *  android.net.wifi.p2p.WifiP2pGroup
 *  android.net.wifi.p2p.WifiP2pInfo
 *  android.net.wifi.p2p.WifiP2pManager
 *  android.net.wifi.p2p.WifiP2pManager$ActionListener
 *  android.net.wifi.p2p.WifiP2pManager$Channel
 *  android.net.wifi.p2p.WifiP2pManager$ChannelListener
 *  android.net.wifi.p2p.WifiP2pManager$ConnectionInfoListener
 *  android.net.wifi.p2p.WifiP2pManager$GroupInfoListener
 *  android.net.wifi.p2p.WifiP2pManager$PeerListListener
 *  android.os.Looper
 *  android.os.Parcelable
 */
package com.qualcomm.robotcore.wifi;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.net.NetworkInfo;
import android.net.wifi.WpsInfo;
import android.net.wifi.p2p.WifiP2pConfig;
import android.net.wifi.p2p.WifiP2pDevice;
import android.net.wifi.p2p.WifiP2pDeviceList;
import android.net.wifi.p2p.WifiP2pGroup;
import android.net.wifi.p2p.WifiP2pInfo;
import android.net.wifi.p2p.WifiP2pManager;
import android.os.Looper;
import android.os.Parcelable;
import com.qualcomm.robotcore.util.RobotLog;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class WifiDirectAssistant {
    private static WifiDirectAssistant a = null;
    private final List<WifiP2pDevice> b = new ArrayList<WifiP2pDevice>();
    private Context c = null;
    private boolean d = false;
    private final IntentFilter e;
    private final WifiP2pManager.Channel f;
    private final WifiP2pManager g;
    private d h;
    private final a i;
    private final c j;
    private final b k;
    private int l = 0;
    private ConnectStatus m = ConnectStatus.NOT_CONNECTED;
    private Event n = null;
    private String o = "";
    private String p = "";
    private InetAddress q = null;
    private String r = "";
    private String s = "";
    private String t = "";
    private boolean u = false;
    private int v = 0;
    private WifiDirectAssistantCallback w = null;

    public static synchronized WifiDirectAssistant getWifiDirectAssistant(Context context) {
        if (a == null) {
            a = new WifiDirectAssistant(context);
        }
        return a;
    }

    private WifiDirectAssistant(Context context) {
        this.c = context;
        this.e = new IntentFilter();
        this.e.addAction("android.net.wifi.p2p.STATE_CHANGED");
        this.e.addAction("android.net.wifi.p2p.PEERS_CHANGED");
        this.e.addAction("android.net.wifi.p2p.CONNECTION_STATE_CHANGE");
        this.e.addAction("android.net.wifi.p2p.THIS_DEVICE_CHANGED");
        this.g = (WifiP2pManager)context.getSystemService("wifip2p");
        this.f = this.g.initialize(context, Looper.getMainLooper(), null);
        this.h = new d();
        this.i = new a();
        this.j = new c();
        this.k = new b();
    }

    public synchronized void enable() {
        ++this.v;
        RobotLog.v("There are " + this.v + " Wifi Direct Assistant Clients (+)");
        if (this.v == 1) {
            RobotLog.v("Enabling Wifi Direct Assistant");
            if (this.h == null) {
                this.h = new d();
            }
            this.c.registerReceiver((BroadcastReceiver)this.h, this.e);
        }
    }

    public synchronized void disable() {
        --this.v;
        RobotLog.v("There are " + this.v + " Wifi Direct Assistant Clients (-)");
        if (this.v == 0) {
            RobotLog.v("Disabling Wifi Direct Assistant");
            this.g.stopPeerDiscovery(this.f, null);
            this.g.cancelConnect(this.f, null);
            try {
                this.c.unregisterReceiver((BroadcastReceiver)this.h);
            }
            catch (IllegalArgumentException var1_1) {
                // empty catch block
            }
            this.n = null;
        }
    }

    public synchronized boolean isEnabled() {
        return this.v > 0;
    }

    public ConnectStatus getConnectStatus() {
        return this.m;
    }

    public List<WifiP2pDevice> getPeers() {
        return new ArrayList<WifiP2pDevice>(this.b);
    }

    public WifiDirectAssistantCallback getCallback() {
        return this.w;
    }

    public void setCallback(WifiDirectAssistantCallback callback) {
        this.w = callback;
    }

    public String getDeviceMacAddress() {
        return this.o;
    }

    public String getDeviceName() {
        return this.p;
    }

    public InetAddress getGroupOwnerAddress() {
        return this.q;
    }

    public String getGroupOwnerMacAddress() {
        return this.r;
    }

    public String getGroupOwnerName() {
        return this.s;
    }

    public String getPassphrase() {
        return this.t;
    }

    public boolean isWifiP2pEnabled() {
        return this.d;
    }

    public boolean isConnected() {
        return this.m == ConnectStatus.CONNECTED || this.m == ConnectStatus.GROUP_OWNER;
    }

    public boolean isGroupOwner() {
        return this.m == ConnectStatus.GROUP_OWNER;
    }

    public boolean isDeviceNameValid() {
        return this.p.matches("^\\p{Graph}+$");
    }

    public void discoverPeers() {
        this.g.discoverPeers(this.f, new WifiP2pManager.ActionListener(){

            public void onSuccess() {
                WifiDirectAssistant.this.a(Event.DISCOVERING_PEERS);
                RobotLog.d("Wifi Direct discovering peers");
            }

            public void onFailure(int reason) {
                String string = WifiDirectAssistant.failureReasonToString(reason);
                WifiDirectAssistant.this.l = reason;
                RobotLog.w("Wifi Direct failure while trying to discover peers - reason: " + string);
                WifiDirectAssistant.this.a(Event.ERROR);
            }
        });
    }

    public void cancelDiscoverPeers() {
        RobotLog.d("Wifi Direct stop discovering peers");
        this.g.stopPeerDiscovery(this.f, null);
    }

    public void createGroup() {
        this.g.createGroup(this.f, new WifiP2pManager.ActionListener(){

            public void onSuccess() {
                WifiDirectAssistant.this.m = ConnectStatus.GROUP_OWNER;
                WifiDirectAssistant.this.a(Event.GROUP_CREATED);
                RobotLog.d("Wifi Direct created group");
            }

            public void onFailure(int reason) {
                if (reason == 2) {
                    RobotLog.d("Wifi Direct cannot create group, does group already exist?");
                } else {
                    String string = WifiDirectAssistant.failureReasonToString(reason);
                    WifiDirectAssistant.this.l = reason;
                    RobotLog.w("Wifi Direct failure while trying to create group - reason: " + string);
                    WifiDirectAssistant.this.m = ConnectStatus.ERROR;
                    WifiDirectAssistant.this.a(Event.ERROR);
                }
            }
        });
    }

    public void removeGroup() {
        this.g.removeGroup(this.f, null);
    }

    public void connect(WifiP2pDevice peer) {
        if (this.m == ConnectStatus.CONNECTING || this.m == ConnectStatus.CONNECTED) {
            RobotLog.d("WifiDirect connection request to " + peer.deviceAddress + " ignored, already connected");
            return;
        }
        RobotLog.d("WifiDirect connecting to " + peer.deviceAddress);
        this.m = ConnectStatus.CONNECTING;
        WifiP2pConfig wifiP2pConfig = new WifiP2pConfig();
        wifiP2pConfig.deviceAddress = peer.deviceAddress;
        wifiP2pConfig.wps.setup = 0;
        wifiP2pConfig.groupOwnerIntent = 1;
        this.g.connect(this.f, wifiP2pConfig, new WifiP2pManager.ActionListener(){

            public void onSuccess() {
                RobotLog.d("WifiDirect connect started");
                WifiDirectAssistant.this.a(Event.CONNECTING);
            }

            public void onFailure(int reason) {
                String string = WifiDirectAssistant.failureReasonToString(reason);
                WifiDirectAssistant.this.l = reason;
                RobotLog.d("WifiDirect connect cannot start - reason: " + string);
                WifiDirectAssistant.this.a(Event.ERROR);
            }
        });
    }

    private void a(WifiP2pDevice wifiP2pDevice) {
        this.p = wifiP2pDevice.deviceName;
        this.o = wifiP2pDevice.deviceAddress;
        RobotLog.v("Wifi Direct device information: " + this.p + " " + this.o);
    }

    public String getFailureReason() {
        return WifiDirectAssistant.failureReasonToString(this.l);
    }

    public static String failureReasonToString(int reason) {
        switch (reason) {
            case 1: {
                return "P2P_UNSUPPORTED";
            }
            case 0: {
                return "ERROR";
            }
            case 2: {
                return "BUSY";
            }
        }
        return "UNKNOWN (reason " + reason + ")";
    }

    private void a(Event event) {
        if (this.n == event && this.n != Event.PEERS_AVAILABLE) {
            return;
        }
        this.n = event;
        if (this.w != null) {
            this.w.onWifiDirectEvent(event);
        }
    }

    private class d
    extends BroadcastReceiver {
        private d() {
        }

        public void onReceive(Context context, Intent intent) {
            String string = intent.getAction();
            if ("android.net.wifi.p2p.STATE_CHANGED".equals(string)) {
                int n = intent.getIntExtra("wifi_p2p_state", -1);
                WifiDirectAssistant.this.d = n == 2;
                RobotLog.d("Wifi Direct state - enabled: " + WifiDirectAssistant.this.d);
            } else if ("android.net.wifi.p2p.PEERS_CHANGED".equals(string)) {
                RobotLog.d("Wifi Direct peers changed");
                WifiDirectAssistant.this.g.requestPeers(WifiDirectAssistant.this.f, (WifiP2pManager.PeerListListener)WifiDirectAssistant.this.j);
            } else if ("android.net.wifi.p2p.CONNECTION_STATE_CHANGE".equals(string)) {
                NetworkInfo networkInfo = (NetworkInfo)intent.getParcelableExtra("networkInfo");
                WifiP2pInfo wifiP2pInfo = (WifiP2pInfo)intent.getParcelableExtra("wifiP2pInfo");
                RobotLog.d("Wifi Direct connection changed - connected: " + networkInfo.isConnected());
                if (networkInfo.isConnected()) {
                    WifiDirectAssistant.this.g.requestConnectionInfo(WifiDirectAssistant.this.f, (WifiP2pManager.ConnectionInfoListener)WifiDirectAssistant.this.i);
                    WifiDirectAssistant.this.g.stopPeerDiscovery(WifiDirectAssistant.this.f, null);
                } else {
                    WifiDirectAssistant.this.m = ConnectStatus.NOT_CONNECTED;
                    if (!WifiDirectAssistant.this.u) {
                        WifiDirectAssistant.this.discoverPeers();
                    }
                    if (WifiDirectAssistant.this.isConnected()) {
                        WifiDirectAssistant.this.a(Event.DISCONNECTED);
                    }
                    WifiDirectAssistant.this.u = wifiP2pInfo.groupFormed;
                }
            } else if ("android.net.wifi.p2p.THIS_DEVICE_CHANGED".equals(string)) {
                RobotLog.d("Wifi Direct this device changed");
                WifiDirectAssistant.this.a((WifiP2pDevice)intent.getParcelableExtra("wifiP2pDevice"));
            }
        }
    }

    private class b
    implements WifiP2pManager.GroupInfoListener {
        private b() {
        }

        public void onGroupInfoAvailable(WifiP2pGroup group) {
            if (group == null) {
                return;
            }
            if (group.isGroupOwner()) {
                WifiDirectAssistant.this.r = WifiDirectAssistant.this.o;
                WifiDirectAssistant.this.s = WifiDirectAssistant.this.p;
            } else {
                WifiP2pDevice wifiP2pDevice = group.getOwner();
                WifiDirectAssistant.this.r = wifiP2pDevice.deviceAddress;
                WifiDirectAssistant.this.s = wifiP2pDevice.deviceName;
            }
            WifiDirectAssistant.this.t = group.getPassphrase();
            WifiDirectAssistant.this.t = WifiDirectAssistant.this.t != null ? WifiDirectAssistant.this.t : "";
            RobotLog.v("Wifi Direct connection information available");
            WifiDirectAssistant.this.a(Event.CONNECTION_INFO_AVAILABLE);
        }
    }

    private class a
    implements WifiP2pManager.ConnectionInfoListener {
        private a() {
        }

        public void onConnectionInfoAvailable(WifiP2pInfo info) {
            WifiDirectAssistant.this.g.requestGroupInfo(WifiDirectAssistant.this.f, (WifiP2pManager.GroupInfoListener)WifiDirectAssistant.this.k);
            WifiDirectAssistant.this.q = info.groupOwnerAddress;
            RobotLog.d("Group owners address: " + WifiDirectAssistant.this.q.toString());
            if (info.groupFormed && info.isGroupOwner) {
                RobotLog.d("Wifi Direct group formed, this device is the group owner (GO)");
                WifiDirectAssistant.this.m = ConnectStatus.GROUP_OWNER;
                WifiDirectAssistant.this.a(Event.CONNECTED_AS_GROUP_OWNER);
            } else if (info.groupFormed) {
                RobotLog.d("Wifi Direct group formed, this device is a client");
                WifiDirectAssistant.this.m = ConnectStatus.CONNECTED;
                WifiDirectAssistant.this.a(Event.CONNECTED_AS_PEER);
            } else {
                RobotLog.d("Wifi Direct group NOT formed, ERROR: " + info.toString());
                WifiDirectAssistant.this.l = 0;
                WifiDirectAssistant.this.m = ConnectStatus.ERROR;
                WifiDirectAssistant.this.a(Event.ERROR);
            }
        }
    }

    private class c
    implements WifiP2pManager.PeerListListener {
        private c() {
        }

        public void onPeersAvailable(WifiP2pDeviceList peerList) {
            WifiDirectAssistant.this.b.clear();
            WifiDirectAssistant.this.b.addAll(peerList.getDeviceList());
            RobotLog.v("Wifi Direct peers found: " + WifiDirectAssistant.this.b.size());
            for (WifiP2pDevice wifiP2pDevice : WifiDirectAssistant.this.b) {
                String string = "    peer: " + wifiP2pDevice.deviceAddress + " " + wifiP2pDevice.deviceName;
                RobotLog.v(string);
            }
            WifiDirectAssistant.this.a(Event.PEERS_AVAILABLE);
        }
    }

    public static enum ConnectStatus {
        NOT_CONNECTED,
        CONNECTING,
        CONNECTED,
        GROUP_OWNER,
        ERROR;
        

        private ConnectStatus() {
        }
    }

    public static enum Event {
        DISCOVERING_PEERS,
        PEERS_AVAILABLE,
        GROUP_CREATED,
        CONNECTING,
        CONNECTED_AS_PEER,
        CONNECTED_AS_GROUP_OWNER,
        DISCONNECTED,
        CONNECTION_INFO_AVAILABLE,
        ERROR;
        

        private Event() {
        }
    }

    public static interface WifiDirectAssistantCallback {
        public void onWifiDirectEvent(Event var1);
    }

}

