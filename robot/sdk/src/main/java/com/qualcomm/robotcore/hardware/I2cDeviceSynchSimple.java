package com.qualcomm.robotcore.hardware;

/** Synchronous register reads and writes to one I2C device. Register numbers are 8-bit. */
public interface I2cDeviceSynchSimple extends HardwareDevice {
    byte read8(int ireg);

    byte[] read(int ireg, int creg);

    void write8(int ireg, int bVal);

    void write8(int ireg, int bVal, I2cWaitControl waitControl);

    void write(int ireg, byte[] data);

    void write(int ireg, byte[] data, I2cWaitControl waitControl);

    void waitForWriteCompletions(I2cWaitControl waitControl);

    void setI2cAddress(I2cAddr newAddress);

    I2cAddr getI2cAddress();

    boolean isArmed();

    void setLogging(boolean enabled);

    boolean getLogging();

    void setLoggingTag(String loggingTag);

    String getLoggingTag();

    void setUserConfiguredName(String name);

    String getUserConfiguredName();
}
