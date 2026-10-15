package com.qualcomm.robotcore.hardware;

/**
 * Base class for I2C device drivers: owns the device client that register reads and writes go
 * through. Subclasses implement {@link #doInitialize()} to configure the chip.
 */
public abstract class I2cDeviceSynchDevice<DEVICE_CLIENT extends I2cDeviceSynchSimple> implements HardwareDevice {
    protected DEVICE_CLIENT deviceClient;
    protected boolean deviceClientIsOwned;
    protected boolean isInitialized;

    protected I2cDeviceSynchDevice(DEVICE_CLIENT deviceClient, boolean deviceClientIsOwned) {
        this.deviceClient = deviceClient;
        this.deviceClientIsOwned = deviceClientIsOwned;
    }

    /** With {@code true}, the driver is initialized as soon as the device is armed (which in the simulator is immediately). */
    protected void registerArmingStateCallback(boolean doInitialize) {
        if (doInitialize && deviceClient.isArmed()) initialize();
    }

    protected abstract boolean doInitialize();

    public synchronized boolean initialize() {
        isInitialized = doInitialize();
        return isInitialized;
    }

    public DEVICE_CLIENT getDeviceClient() {
        return deviceClient;
    }

    @Override
    public String getConnectionInfo() {
        return deviceClient.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return deviceClient.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        deviceClient.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        if (deviceClientIsOwned) deviceClient.close();
    }
}
