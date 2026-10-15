package com.qualcomm.robotcore.hardware;

import java.util.Locale;

/** An I2C address. Stored in 7-bit form; the 8-bit form is the 7-bit address shifted left by one. */
public class I2cAddr {
    private final int i2cAddr7Bit;

    public I2cAddr(int i2cAddr7Bit) {
        this.i2cAddr7Bit = i2cAddr7Bit & 0x7f;
    }

    public static I2cAddr create7bit(int i2cAddr7Bit) {
        return new I2cAddr(i2cAddr7Bit);
    }

    public static I2cAddr create8bit(int i2cAddr8Bit) {
        return new I2cAddr(i2cAddr8Bit / 2);
    }

    public static I2cAddr zero() {
        return create7bit(0);
    }

    public boolean isDefault() {
        return i2cAddr7Bit == 0;
    }

    public int get7Bit() {
        return i2cAddr7Bit;
    }

    public int get8Bit() {
        return i2cAddr7Bit * 2;
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof I2cAddr && ((I2cAddr) o).i2cAddr7Bit == i2cAddr7Bit;
    }

    @Override
    public int hashCode() {
        return i2cAddr7Bit;
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "I2cAddr(0x%02x)", i2cAddr7Bit);
    }
}
