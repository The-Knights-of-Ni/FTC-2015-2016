/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.robocol;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robocol.RobocolParsable;
import com.qualcomm.robotcore.robocol.RobocolParsableBase;
import com.qualcomm.robotcore.util.TypeConversion;
import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

public class Telemetry
extends RobocolParsableBase {
    public static final String DEFAULT_TAG = "TELEMETRY_DATA";
    private static final Charset a = Charset.forName("UTF-8");
    private final Map<String, String> b = new LinkedHashMap<String, String>();
    private final Map<String, Float> c = new LinkedHashMap<String, Float>();
    private String d = "";
    private long e = 0;
    private boolean f = true;
    public static final int cbTagMax = 255;
    public static final int cCountMax = 255;
    public static final int cbKeyMax = 65535;
    public static final int cbValueMax = 65535;

    public Telemetry() {
    }

    public Telemetry(byte[] byteArray) throws RobotCoreException {
        this.fromByteArray(byteArray);
    }

    public synchronized long getTimestamp() {
        return this.e;
    }

    public boolean isSorted() {
        return this.f;
    }

    public void setSorted(boolean isSorted) {
        this.f = isSorted;
    }

    public synchronized void setTag(String tag) {
        this.d = tag;
    }

    public synchronized String getTag() {
        if (this.d.length() == 0) {
            return "TELEMETRY_DATA";
        }
        return this.d;
    }

    public synchronized void addData(String key, String msg) {
        this.b.put(key, msg);
    }

    public synchronized void addData(String key, Object msg) {
        this.b.put(key, msg.toString());
    }

    public synchronized void addData(String key, float msg) {
        this.c.put(key, Float.valueOf(msg));
    }

    public synchronized void addData(String key, double msg) {
        this.c.put(key, Float.valueOf((float)msg));
    }

    public synchronized Map<String, String> getDataStrings() {
        return this.b;
    }

    public synchronized Map<String, Float> getDataNumbers() {
        return this.c;
    }

    public synchronized boolean hasData() {
        return !this.b.isEmpty() || !this.c.isEmpty();
    }

    public synchronized void clearData() {
        this.e = 0;
        this.b.clear();
        this.c.clear();
    }

    @Override
    public RobocolParsable.MsgType getRobocolMsgType() {
        return RobocolParsable.MsgType.TELEMETRY;
    }

    @Override
    public synchronized byte[] toByteArray() throws RobotCoreException {
        byte[] arrby;
        this.e = System.currentTimeMillis();
        if (this.b.size() > 255) {
            throw new RobotCoreException("Cannot have more than %d string data points", 255);
        }
        if (this.c.size() > 255) {
            throw new RobotCoreException("Cannot have more than %d number data points", 255);
        }
        int n = this.a();
        int n2 = 5 + n;
        if (n2 > 4098) {
            throw new RobotCoreException(String.format("Cannot send telemetry data of %d bytes; max is %d", n2, 4098));
        }
        ByteBuffer byteBuffer = this.getWriteBuffer(n);
        byteBuffer.putLong(this.e);
        byteBuffer.put(this.f ? 1 : 0);
        if (this.d.length() == 0) {
            Telemetry.b(byteBuffer, 0);
        } else {
            byte[] iterator = this.d.getBytes(a);
            if (iterator.length > 255) {
                throw new RobotCoreException(String.format("Telemetry tag cannot exceed %d bytes [%s]", 255, this.d));
            }
            Telemetry.b(byteBuffer, iterator.length);
            byteBuffer.put(iterator);
        }
        Telemetry.a(byteBuffer, this.b.size());
        for (Map.Entry entry2 : this.b.entrySet()) {
            arrby = entry2.getKey().getBytes(a);
            byte[] arrby2 = entry2.getValue().getBytes(a);
            if (arrby.length > 65535) {
                throw new RobotCoreException("telemetry key '%s' too long: %d bytes; max %d bytes", entry2.getKey(), arrby.length, 65535);
            }
            if (arrby2.length > 65535) {
                throw new RobotCoreException("telemetry value '%s' too long: %d bytes; max %d bytes", entry2.getValue(), arrby2.length, 65535);
            }
            Telemetry.c(byteBuffer, arrby.length);
            byteBuffer.put(arrby);
            Telemetry.d(byteBuffer, arrby2.length);
            byteBuffer.put(arrby2);
        }
        Telemetry.a(byteBuffer, this.c.size());
        for (Map.Entry entry2 : this.c.entrySet()) {
            arrby = ((String)entry2.getKey()).getBytes(a);
            float f = ((Float)entry2.getValue()).floatValue();
            if (arrby.length > 65535) {
                throw new RobotCoreException("telemetry key '%s' too long: %d bytes; max %d bytes", entry2.getKey(), arrby.length, 65535);
            }
            Telemetry.c(byteBuffer, arrby.length);
            byteBuffer.put(arrby);
            byteBuffer.putFloat(f);
        }
        return byteBuffer.array();
    }

    @Override
    public synchronized void fromByteArray(byte[] byteArray) throws RobotCoreException {
        int n;
        int n2;
        Object object;
        this.clearData();
        ByteBuffer byteBuffer = this.getReadBuffer(byteArray);
        this.e = byteBuffer.getLong();
        this.f = byteBuffer.get() != 0;
        int n3 = Telemetry.b(byteBuffer);
        if (n3 == 0) {
            this.d = "";
        } else {
            byte[] arrby = new byte[n3];
            byteBuffer.get(arrby);
            this.d = new String(arrby, a);
        }
        int n4 = Telemetry.a(byteBuffer);
        for (n2 = 0; n2 < n4; ++n2) {
            n = Telemetry.c(byteBuffer);
            byte[] arrby = new byte[n];
            byteBuffer.get(arrby);
            int n5 = Telemetry.d(byteBuffer);
            object = new byte[n5];
            byteBuffer.get((byte[])object);
            String string = new String(arrby, a);
            String string2 = new String((byte[])object, a);
            this.b.put(string, string2);
        }
        n2 = Telemetry.a(byteBuffer);
        for (n = 0; n < n2; ++n) {
            int n6 = Telemetry.c(byteBuffer);
            byte[] arrby = new byte[n6];
            byteBuffer.get(arrby);
            object = new String(arrby, a);
            float f = byteBuffer.getFloat();
            this.c.put((String)object, Float.valueOf(f));
        }
    }

    static void a(ByteBuffer byteBuffer, int n) {
        byteBuffer.put((byte)n);
    }

    static int a(ByteBuffer byteBuffer) {
        return TypeConversion.unsignedByteToInt(byteBuffer.get());
    }

    static void b(ByteBuffer byteBuffer, int n) {
        byteBuffer.put((byte)n);
    }

    static int b(ByteBuffer byteBuffer) {
        return TypeConversion.unsignedByteToInt(byteBuffer.get());
    }

    static void c(ByteBuffer byteBuffer, int n) {
        byteBuffer.putShort((short)n);
    }

    static int c(ByteBuffer byteBuffer) {
        return TypeConversion.unsignedShortToInt(byteBuffer.getShort());
    }

    static void d(ByteBuffer byteBuffer, int n) {
        Telemetry.c(byteBuffer, n);
    }

    static int d(ByteBuffer byteBuffer) {
        return Telemetry.c(byteBuffer);
    }

    private int a() {
        int n = 9;
        n+=1 + this.d.getBytes(a).length;
        ++n;
        for (Map.Entry<String, String> entry2 : this.b.entrySet()) {
            n+=2 + entry2.getKey().getBytes(a).length;
            n+=2 + entry2.getValue().getBytes(a).length;
        }
        ++n;
        for (Map.Entry entry : this.c.entrySet()) {
            n+=2 + ((String)entry.getKey()).getBytes(a).length;
            n+=4;
        }
        return n;
    }
}

