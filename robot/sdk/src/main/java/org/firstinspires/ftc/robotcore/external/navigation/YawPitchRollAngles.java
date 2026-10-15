package org.firstinspires.ftc.robotcore.external.navigation;

import java.util.Locale;

/** Robot orientation: yaw is counterclockwise-positive about up, pitch is about the right axis, roll about forward. */
public class YawPitchRollAngles {
    private final AngleUnit angleUnit;
    private final double yaw, pitch, roll;
    private final long acquisitionTime;

    public YawPitchRollAngles(AngleUnit angleUnit, double yaw, double pitch, double roll, long acquisitionTime) {
        this.angleUnit = angleUnit;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
        this.acquisitionTime = acquisitionTime;
    }

    public double getYaw(AngleUnit unit) {
        return unit.fromUnit(angleUnit, yaw);
    }

    public double getPitch(AngleUnit unit) {
        return unit.fromUnit(angleUnit, pitch);
    }

    public double getRoll(AngleUnit unit) {
        return unit.fromUnit(angleUnit, roll);
    }

    /** Degrees. */
    public double getYaw() {
        return getYaw(AngleUnit.DEGREES);
    }

    /** Degrees. */
    public double getPitch() {
        return getPitch(AngleUnit.DEGREES);
    }

    /** Degrees. */
    public double getRoll() {
        return getRoll(AngleUnit.DEGREES);
    }

    public long getAcquisitionTime() {
        return acquisitionTime;
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "{yaw=%.3f, pitch=%.3f, roll=%.3f}", getYaw(), getPitch(), getRoll());
    }
}
