/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.robocol;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robocol.RobocolParsable;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

public class RobocolDatagram {
    private DatagramPacket b;
    private byte[] c = null;
    static Queue<byte[]> a = new ConcurrentLinkedQueue<byte[]>();

    public RobocolDatagram(RobocolParsable message) throws RobotCoreException {
        this.setData(message.toByteArrayForTransmission());
    }

    public RobocolDatagram(byte[] message) {
        this.setData(message);
    }

    public static RobocolDatagram forReceive() {
        byte[] arrby = a.poll();
        if (arrby == null) {
            arrby = new byte[4098];
        }
        DatagramPacket datagramPacket = new DatagramPacket(arrby, arrby.length);
        RobocolDatagram robocolDatagram = new RobocolDatagram();
        robocolDatagram.b = datagramPacket;
        robocolDatagram.c = arrby;
        return robocolDatagram;
    }

    protected RobocolDatagram() {
        this.b = null;
    }

    public void close() {
        if (this.c != null) {
            a.add(this.c);
            this.c = null;
        }
        this.b = null;
    }

    public RobocolParsable.MsgType getMsgType() {
        return RobocolParsable.MsgType.fromByte(this.b.getData()[0]);
    }

    public int getLength() {
        return this.b.getLength();
    }

    public int getPayloadLength() {
        return this.b.getLength() - 5;
    }

    public byte[] getData() {
        return this.b.getData();
    }

    public void setData(byte[] data) {
        this.b = new DatagramPacket(data, data.length);
    }

    public InetAddress getAddress() {
        return this.b.getAddress();
    }

    public void setAddress(InetAddress address) {
        this.b.setAddress(address);
    }

    public String toString() {
        int n = 0;
        String string = "NONE";
        String string2 = null;
        if (this.b != null && this.b.getAddress() != null && this.b.getLength() > 0) {
            string = RobocolParsable.MsgType.fromByte(this.b.getData()[0]).name();
            n = this.b.getLength();
            string2 = this.b.getAddress().getHostAddress();
        }
        return String.format("RobocolDatagram - type:%s, addr:%s, size:%d", string, string2, n);
    }

    protected DatagramPacket getPacket() {
        return this.b;
    }

    protected void setPacket(DatagramPacket packet) {
        this.b = packet;
    }
}

