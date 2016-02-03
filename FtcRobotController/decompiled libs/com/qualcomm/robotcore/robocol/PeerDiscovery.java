/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.robocol;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robocol.RobocolParsable;
import com.qualcomm.robotcore.robocol.RobocolParsableBase;
import com.qualcomm.robotcore.util.RobotLog;
import java.nio.BufferOverflowException;
import java.nio.ByteBuffer;

public class PeerDiscovery
extends RobocolParsableBase {
    public static final byte ROBOCOL_VERSION = 2;
    private PeerType a;

    private PeerDiscovery() {
        this.a = PeerType.NOT_SET;
    }

    public static PeerDiscovery forReceive() {
        return new PeerDiscovery();
    }

    public PeerDiscovery(PeerType peerType) {
        this.a = peerType;
    }

    public PeerType getPeerType() {
        return this.a;
    }

    @Override
    public RobocolParsable.MsgType getRobocolMsgType() {
        return RobocolParsable.MsgType.PEER_DISCOVERY;
    }

    @Override
    public byte[] toByteArray() throws RobotCoreException {
        ByteBuffer byteBuffer = this.allocateWholeWriteBuffer(13);
        try {
            byteBuffer.put(this.getRobocolMsgType().asByte());
            byteBuffer.putShort(10);
            byteBuffer.put(2);
            byteBuffer.put(this.a.asByte());
            byteBuffer.putShort((short)this.sequenceNumber);
        }
        catch (BufferOverflowException var2_2) {
            RobotLog.logStacktrace(var2_2);
        }
        return byteBuffer.array();
    }

    @Override
    public void fromByteArray(byte[] byteArray) throws RobotCoreException {
        if (byteArray.length < 13) {
            throw new RobotCoreException("Expected buffer of at least %d bytes, received %d", 13, byteArray.length);
        }
        ByteBuffer byteBuffer = this.getWholeReadBuffer(byteArray);
        byte by = byteBuffer.get();
        short s = byteBuffer.getShort();
        byte by2 = byteBuffer.get();
        byte by3 = byteBuffer.get();
        short s2 = byteBuffer.getShort();
        if (by2 != 2) {
            throw new RobotCoreException("Incompatible apps: v%d vs v%d", Byte.valueOf(2), Byte.valueOf(by2));
        }
        this.a = PeerType.fromByte(by3);
        if (by2 > 1) {
            this.setSequenceNumber(s2);
        }
    }

    public String toString() {
        return String.format("Peer Discovery - peer type: %s", this.a.name());
    }

    public static enum PeerType {
        NOT_SET(0),
        PEER(1),
        GROUP_OWNER(2);
        
        private static final PeerType[] a;
        private int b;

        public static PeerType fromByte(byte b) {
            PeerType peerType = NOT_SET;
            try {
                peerType = a[b];
            }
            catch (ArrayIndexOutOfBoundsException var2_2) {
                RobotLog.w(String.format("Cannot convert %d to Peer: %s", Byte.valueOf(b), var2_2.toString()));
            }
            return peerType;
        }

        private PeerType(int type) {
            this.b = type;
        }

        public byte asByte() {
            return (byte)this.b;
        }

        static {
            a = PeerType.values();
        }
    }

}

