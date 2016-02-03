/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.robocol;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robocol.PeerDiscovery;
import com.qualcomm.robotcore.robocol.RobocolDatagram;
import com.qualcomm.robotcore.robocol.RobocolDatagramSocket;
import com.qualcomm.robotcore.robocol.RobocolParsable;
import com.qualcomm.robotcore.util.RobotLog;
import java.net.InetAddress;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class PeerDiscoveryManager {
    private InetAddress a;
    private final RobocolDatagramSocket b;
    private ScheduledExecutorService c;
    private ScheduledFuture<?> d;
    private final PeerDiscovery e = new PeerDiscovery(PeerDiscovery.PeerType.PEER);

    public PeerDiscoveryManager(RobocolDatagramSocket socket) {
        this.b = socket;
    }

    public InetAddress getPeerDiscoveryDevice() {
        return this.a;
    }

    public void start(InetAddress peerDiscoveryDevice) {
        RobotLog.v("Starting peer discovery");
        if (peerDiscoveryDevice == this.b.getLocalAddress()) {
            RobotLog.v("No need for peer discovery, we are the peer discovery device");
            return;
        }
        if (this.d != null) {
            this.d.cancel(true);
        }
        this.a = peerDiscoveryDevice;
        this.c = Executors.newSingleThreadScheduledExecutor();
        this.d = this.c.scheduleAtFixedRate(new a(), 1, 1, TimeUnit.SECONDS);
    }

    public void stop() {
        RobotLog.v("Stopping peer discovery");
        if (this.d != null) {
            this.d.cancel(true);
        }
    }

    private class a
    implements Runnable {
        private a() {
        }

        @Override
        public void run() {
            try {
                RobotLog.v("sending peer discovery packet(%d)", PeerDiscoveryManager.this.e.getSequenceNumber());
                RobocolDatagram robocolDatagram = new RobocolDatagram(PeerDiscoveryManager.this.e);
                if (PeerDiscoveryManager.this.b.getInetAddress() == null) {
                    robocolDatagram.setAddress(PeerDiscoveryManager.this.a);
                }
                PeerDiscoveryManager.this.b.send(robocolDatagram);
            }
            catch (RobotCoreException var1_2) {
                RobotLog.d("Unable to send peer discovery packet: " + var1_2.toString());
            }
        }
    }

}

