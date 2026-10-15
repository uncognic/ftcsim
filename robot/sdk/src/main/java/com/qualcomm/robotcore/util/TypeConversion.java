package com.qualcomm.robotcore.util;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;

/** Conversions between numbers and byte arrays, used by device drivers. Default byte order is big-endian. */
public class TypeConversion {
    private TypeConversion() {
    }

    public static byte[] shortToByteArray(short shortInt) {
        return shortToByteArray(shortInt, ByteOrder.BIG_ENDIAN);
    }

    public static byte[] shortToByteArray(short shortInt, ByteOrder byteOrder) {
        return ByteBuffer.allocate(2).order(byteOrder).putShort(shortInt).array();
    }

    public static byte[] intToByteArray(int integer) {
        return intToByteArray(integer, ByteOrder.BIG_ENDIAN);
    }

    public static byte[] intToByteArray(int integer, ByteOrder byteOrder) {
        return ByteBuffer.allocate(4).order(byteOrder).putInt(integer).array();
    }

    public static byte[] longToByteArray(long longInt) {
        return longToByteArray(longInt, ByteOrder.BIG_ENDIAN);
    }

    public static byte[] longToByteArray(long longInt, ByteOrder byteOrder) {
        return ByteBuffer.allocate(8).order(byteOrder).putLong(longInt).array();
    }

    public static short byteArrayToShort(byte[] byteArray) {
        return byteArrayToShort(byteArray, ByteOrder.BIG_ENDIAN);
    }

    public static short byteArrayToShort(byte[] byteArray, ByteOrder byteOrder) {
        return ByteBuffer.wrap(byteArray).order(byteOrder).getShort();
    }

    public static short byteArrayToShort(byte[] byteArray, int ibFirst, ByteOrder byteOrder) {
        return ByteBuffer.wrap(byteArray, ibFirst, 2).order(byteOrder).getShort();
    }

    public static int byteArrayToInt(byte[] byteArray) {
        return byteArrayToInt(byteArray, ByteOrder.BIG_ENDIAN);
    }

    public static int byteArrayToInt(byte[] byteArray, ByteOrder byteOrder) {
        return ByteBuffer.wrap(byteArray).order(byteOrder).getInt();
    }

    public static long byteArrayToLong(byte[] byteArray) {
        return byteArrayToLong(byteArray, ByteOrder.BIG_ENDIAN);
    }

    public static long byteArrayToLong(byte[] byteArray, ByteOrder byteOrder) {
        return ByteBuffer.wrap(byteArray).order(byteOrder).getLong();
    }

    public static int unsignedByteToInt(byte b) {
        return b & 0xff;
    }

    public static double unsignedByteToDouble(byte b) {
        return b & 0xff;
    }

    public static long unsignedIntToLong(int i) {
        return i & 0xffffffffL;
    }

    public static int unsignedShortToInt(short s) {
        return s & 0xffff;
    }

    public static byte[] stringToUtf8(String javaString) {
        return javaString.getBytes(StandardCharsets.UTF_8);
    }

    public static String utf8ToString(byte[] utf8) {
        return new String(utf8, StandardCharsets.UTF_8);
    }
}
