package org.ftcsim.runner;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.ftcsim.sdk.SimBus;

/**
 * The Control Hub's built-in IMU, reporting in robot axes. For now the simulator assumes the
 * orientation passed to {@link #initialize} matches how the hub is mounted.
 */
final class SimImu implements IMU {
    /** An I2C read takes longer than a motor command. */
    private static final int READ_COST = 2;

    private final String name;
    private volatile double yawOffset;

    volatile double yaw, pitch, roll;
    volatile float rateX, rateY, rateZ;

    SimImu(String name) {
        this.name = name;
    }

    @Override
    public boolean initialize(Parameters parameters) {
        SimBus.transaction(READ_COST);
        return true;
    }

    @Override
    public void resetYaw() {
        SimBus.transaction(1);
        yawOffset = yaw;
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        SimBus.transaction(READ_COST);
        return new YawPitchRollAngles(AngleUnit.RADIANS, AngleUnit.normalizeRadians(yaw - yawOffset), pitch, roll, SimBus.nanoTime());
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        SimBus.transaction(READ_COST);
        return new AngularVelocity(AngleUnit.RADIANS, rateX, rateY, rateZ, SimBus.nanoTime()).toAngleUnit(angleUnit);
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
        return "Simulated BHI260AP IMU " + name;
    }

    @Override
    public String getConnectionInfo() {
        return "Control Hub; I2C bus 0";
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void close() {
    }
}
