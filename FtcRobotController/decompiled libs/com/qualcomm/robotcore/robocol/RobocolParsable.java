/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.robocol;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.RobotLog;

public interface RobocolParsable {
    public static final int HEADER_LENGTH = 5;

    public MsgType getRobocolMsgType();

    public int getSequenceNumber();

    public void setSequenceNumber();

    public boolean shouldTransmit(long var1);

    public byte[] toByteArrayForTransmission() throws RobotCoreException;

    public byte[] toByteArray() throws RobotCoreException;

    public void fromByteArray(byte[] var1) throws RobotCoreException;

    public static enum MsgType {
        EMPTY(0),
        HEARTBEAT(1),
        GAMEPAD(2),
        PEER_DISCOVERY(3),
        COMMAND(4),
        TELEMETRY(5);
        
        private static final MsgType[] a;
        private final int b;

        public static MsgType fromByte(byte b) {
            MsgType msgType = EMPTY;
            try {
                msgType = a[b];
            }
            catch (ArrayIndexOutOfBoundsException var2_2) {
                RobotLog.w(String.format("Cannot convert %d to MsgType: %s", Byte.valueOf(b), var2_2.toString()));
            }
            return msgType;
        }

        private MsgType(int type) {
            this.b = type;
        }

        public byte asByte() {
            return (byte)this.b;
        }

        static {
            a = MsgType.values();
        }
    }

}

