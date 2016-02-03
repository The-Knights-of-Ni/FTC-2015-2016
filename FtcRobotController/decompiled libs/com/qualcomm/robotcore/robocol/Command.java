/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.robocol;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robocol.RobocolParsable;
import com.qualcomm.robotcore.robocol.RobocolParsableBase;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;
import java.nio.BufferOverflowException;
import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.util.Comparator;

public class Command
extends RobocolParsableBase
implements Comparable<Command>,
Comparator<Command> {
    private static final Charset h = Charset.forName("UTF-8");
    String a;
    String b;
    byte[] c;
    byte[] d;
    long e;
    boolean f = false;
    byte g = 0;

    public Command(String name) {
        this(name, "");
    }

    public Command(String name, String extra) {
        this.a = name;
        this.b = extra;
        this.c = TypeConversion.stringToUtf8(this.a);
        this.d = TypeConversion.stringToUtf8(this.b);
        this.e = Command.generateTimestamp();
        int n = this.a();
        if (n > 32767) {
            throw new IllegalArgumentException(String.format("command payload is too large: %d", n));
        }
    }

    public Command(byte[] byteArray) throws RobotCoreException {
        this.fromByteArray(byteArray);
    }

    public void acknowledge() {
        this.f = true;
    }

    public boolean isAcknowledged() {
        return this.f;
    }

    public String getName() {
        return this.a;
    }

    public String getExtra() {
        return this.b;
    }

    public byte getAttempts() {
        return this.g;
    }

    public long getTimestamp() {
        return this.e;
    }

    @Override
    public RobocolParsable.MsgType getRobocolMsgType() {
        return RobocolParsable.MsgType.COMMAND;
    }

    @Override
    public byte[] toByteArray() throws RobotCoreException {
        if (this.g != 127) {
            this.g = (byte)(this.g + 1);
        }
        short s = (short)this.a();
        ByteBuffer byteBuffer = this.getWriteBuffer(s);
        try {
            byteBuffer.putLong(this.e);
            byteBuffer.put(this.f ? 1 : 0);
            byteBuffer.putShort((short)this.c.length);
            byteBuffer.put(this.c);
            byteBuffer.putShort((short)this.d.length);
            byteBuffer.put(this.d);
        }
        catch (BufferOverflowException var3_3) {
            RobotLog.logStacktrace(var3_3);
        }
        return byteBuffer.array();
    }

    int a() {
        return 13 + this.c.length + this.d.length;
    }

    @Override
    public void fromByteArray(byte[] byteArray) throws RobotCoreException {
        ByteBuffer byteBuffer = this.getReadBuffer(byteArray);
        this.e = byteBuffer.getLong();
        this.f = byteBuffer.get() != 0;
        int n = TypeConversion.unsignedShortToInt(byteBuffer.getShort());
        this.c = new byte[n];
        byteBuffer.get(this.c);
        this.a = TypeConversion.utf8ToString(this.c);
        n = TypeConversion.unsignedShortToInt(byteBuffer.getShort());
        this.d = new byte[n];
        byteBuffer.get(this.d);
        this.b = TypeConversion.utf8ToString(this.d);
    }

    public String toString() {
        return String.format("command: %20d %5s %s", this.e, this.f, this.a);
    }

    @Override
    public boolean equals(Object o) {
        if (o instanceof Command) {
            Command command = (Command)o;
            if (this.a.equals(command.a) && this.e == command.e) {
                return true;
            }
        }
        return false;
    }

    public int hashCode() {
        return this.a.hashCode() ^ (int)this.e;
    }

    @Override
    public int compareTo(Command another) {
        int n = this.a.compareTo(another.a);
        if (n != 0) {
            return n;
        }
        if (this.e < another.e) {
            return -1;
        }
        if (this.e > another.e) {
            return 1;
        }
        return 0;
    }

    @Override
    public int compare(Command c1, Command c2) {
        return c1.compareTo(c2);
    }

    public static long generateTimestamp() {
        return System.nanoTime();
    }
}

