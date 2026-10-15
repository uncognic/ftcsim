package org.ftcsim.runner;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.ftcsim.sdk.SimBus;

/** The hub's battery voltage reading. */
final class SimVoltageSensor implements VoltageSensor {
    volatile double voltage = 13.0;

    @Override
    public double getVoltage() {
        SimBus.transaction(1);
        return voltage;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Lynx;
    }

    @Override
    public String getDeviceName() {
        return "Simulated hub voltage sensor";
    }

    @Override
    public String getConnectionInfo() {
        return "Control Hub";
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void close() {
    }
}
