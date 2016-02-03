/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.util;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.Charset;

public class TypeConversion {
    private static final Charset a = Charset.forName("UTF-8");

    private TypeConversion() {
    }

    public static byte[] shortToByteArray(short shortInt) {
        return TypeConversion.shortToByteArray(shortInt, ByteOrder.BIG_ENDIAN);
    }

    public static byte[] shortToByteArray(short shortInt, ByteOrder byteOrder) {
        return ByteBuffer.allocate(2).order(byteOrder).putShort(shortInt).array();
    }

    public static byte[] intToByteArray(int integer) {
        return TypeConversion.intToByteArray(integer, ByteOrder.BIG_ENDIAN);
    }

    public static byte[] intToByteArray(int integer, ByteOrder byteOrder) {
        return ByteBuffer.allocate(4).order(byteOrder).putInt(integer).array();
    }

    public static byte[] longToByteArray(long longInt) {
        return TypeConversion.longToByteArray(longInt, ByteOrder.BIG_ENDIAN);
    }

    public static byte[] longToByteArray(long longInt, ByteOrder byteOrder) {
        return ByteBuffer.allocate(8).order(byteOrder).putLong(longInt).array();
    }

    public static short byteArrayToShort(byte[] byteArray) {
        return TypeConversion.byteArrayToShort(byteArray, ByteOrder.BIG_ENDIAN);
    }

    public static short byteArrayToShort(byte[] byteArray, ByteOrder byteOrder) {
        return ByteBuffer.wrap(byteArray).order(byteOrder).getShort();
    }

    public static int byteArrayToInt(byte[] byteArray) {
        return TypeConversion.byteArrayToInt(byteArray, ByteOrder.BIG_ENDIAN);
    }

    public static int byteArrayToInt(byte[] byteArray, ByteOrder byteOrder) {
        return ByteBuffer.wrap(byteArray).order(byteOrder).getInt();
    }

    public static long byteArrayToLong(byte[] byteArray) {
        return TypeConversion.byteArrayToLong(byteArray, ByteOrder.BIG_ENDIAN);
    }

    public static long byteArrayToLong(byte[] byteArray, ByteOrder byteOrder) {
        return ByteBuffer.wrap(byteArray).order(byteOrder).getLong();
    }

    public static int unsignedByteToInt(byte b) {
        return b & 255;
    }

    public static int unsignedShortToInt(short s) {
        return s & 65535;
    }

    public static double unsignedByteToDouble(byte b) {
        return b & 255;
    }

    public static long unsignedIntToLong(int i) {
        return (long)i & 0xFFFFFFFFL;
    }

    public static byte[] stringToUtf8(String javaString) {
        byte[] arrby = javaString.getBytes(a);
        if (!javaString.equals(new String(arrby, a))) {
            String string = String.format("string cannot be cleanly encoded into %s - '%s' -> '%s'", a.name(), javaString, new String(arrby, a));
            throw new IllegalArgumentException(string);
        }
        return arrby;
    }

    public static String utf8ToString(byte[] utf8String) {
        return new String(utf8String, a);
    }
}

