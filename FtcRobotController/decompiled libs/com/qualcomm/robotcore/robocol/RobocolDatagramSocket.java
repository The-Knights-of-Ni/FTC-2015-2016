/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.robocol;

import com.qualcomm.robotcore.robocol.RobocolConfig;
import com.qualcomm.robotcore.robocol.RobocolDatagram;
import com.qualcomm.robotcore.util.RobotLog;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.PortUnreachableException;
import java.net.SocketAddress;
import java.net.SocketException;

public class RobocolDatagramSocket {
    private DatagramSocket a;
    private volatile State b = State.CLOSED;
    private final Object c = new Object();
    private final Object d = new Object();
    private final Object e = new Object();

    public void listen(InetAddress destAddress) throws SocketException {
        this.bind(new InetSocketAddress(RobocolConfig.determineBindAddress(destAddress), 20884));
    }

    public void bind(InetSocketAddress bindAddress) throws SocketException {
        Object object = this.e;
        synchronized (object) {
            if (this.b != State.CLOSED) {
                this.close();
            }
            this.b = State.LISTENING;
            RobotLog.d("RobocolDatagramSocket binding to " + bindAddress.toString());
            this.a = new DatagramSocket(bindAddress);
        }
    }

    public void connect(InetAddress connectAddress) throws SocketException {
        InetSocketAddress inetSocketAddress = new InetSocketAddress(connectAddress, 20884);
        RobotLog.d("RobocolDatagramSocket connected to " + inetSocketAddress.toString());
        this.a.connect(inetSocketAddress);
    }

    public void close() {
        Object object = this.e;
        synchronized (object) {
            this.b = State.CLOSED;
            if (this.a != null) {
                this.a.close();
            }
            RobotLog.d("RobocolDatagramSocket is closed");
        }
    }

    public void send(RobocolDatagram message) {
        Object object = this.d;
        synchronized (object) {
            try {
                this.a.send(message.getPacket());
            }
            catch (IllegalArgumentException var3_3) {
                RobotLog.w("Unable to send RobocolDatagram: " + var3_3.toString());
                RobotLog.w("               " + message.toString());
            }
            catch (IOException var3_4) {
                RobotLog.w("Unable to send RobocolDatagram: " + var3_4.toString());
                RobotLog.w("               " + message.toString());
            }
            catch (NullPointerException var3_5) {
                RobotLog.w("Unable to send RobocolDatagram: " + var3_5.toString());
                RobotLog.w("               " + message.toString());
            }
        }
    }

    public RobocolDatagram recv() {
        Object object = this.c;
        synchronized (object) {
            RobocolDatagram robocolDatagram = RobocolDatagram.forReceive();
            DatagramPacket datagramPacket = robocolDatagram.getPacket();
            try {
                this.a.receive(datagramPacket);
            }
            catch (PortUnreachableException var4_4) {
                RobotLog.d("RobocolDatagramSocket receive error: remote port unreachable");
                return null;
            }
            catch (IOException var4_5) {
                RobotLog.d("RobocolDatagramSocket receive error: " + var4_5.toString());
                return null;
            }
            catch (NullPointerException var4_6) {
                RobotLog.d("RobocolDatagramSocket receive error: " + var4_6.toString());
            }
            return robocolDatagram;
        }
    }

    public State getState() {
        return this.b;
    }

    public InetAddress getInetAddress() {
        if (this.a == null) {
            return null;
        }
        return this.a.getInetAddress();
    }

    public InetAddress getLocalAddress() {
        if (this.a == null) {
            return null;
        }
        return this.a.getLocalAddress();
    }

    public boolean isRunning() {
        return this.b == State.LISTENING;
    }

    public boolean isClosed() {
        return this.b == State.CLOSED;
    }

    public static enum State {
        LISTENING,
        CLOSED,
        ERROR;
        

        private State() {
        }
    }

}

