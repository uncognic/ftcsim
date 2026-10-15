package org.ftcsim.runner;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import java.lang.reflect.Constructor;
import java.util.HashMap;
import java.util.Map;

/**
 * Adds what a configured I2C port does on a real robot: asking for a driver class by name
 * builds that driver on the port's device. This is how a team's own driver (for example its
 * copy of the Pinpoint driver) talks to the same simulated chip as the SDK's driver.
 */
final class SimHardwareMap extends HardwareMap {
    private final Map<String, I2cDeviceSynchSimple> i2cDevices = new HashMap<>();

    void putI2c(String name, I2cDeviceSynchSimple device) {
        i2cDevices.put(name.trim(), device);
    }

    @Override
    public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        T found = super.tryGet(classOrInterface, deviceName);
        if (found != null) return found;
        I2cDeviceSynchSimple client = i2cDevices.get(deviceName.trim());
        if (client == null) return null;
        if (classOrInterface.isInstance(client)) {
            @SuppressWarnings("unchecked") T t = (T) client;
            return t;
        }
        if (!I2cDeviceSynchDevice.class.isAssignableFrom(classOrInterface)) return null;
        try {
            Constructor<? extends T> ctor = classOrInterface.getConstructor(I2cDeviceSynchSimple.class, boolean.class);
            T driver = ctor.newInstance(client, false);
            put(deviceName, (com.qualcomm.robotcore.hardware.HardwareDevice) driver);
            return driver;
        } catch (ReflectiveOperationException e) {
            throw new IllegalArgumentException("Could not create I2C driver " + classOrInterface.getName() + " for \"" + deviceName
                    + "\": it needs a public (I2cDeviceSynchSimple, boolean) constructor", e);
        }
    }
}
