package org.firstinspires.ftc.robotcore.external.navigation;

/** Like {@link AngleUnit}, but values are never wrapped into one revolution (used for rates and accumulated angles). */
public enum UnnormalizedAngleUnit {
    DEGREES, RADIANS;

    public double fromRadians(double radians) {
        return this == RADIANS ? radians : Math.toDegrees(radians);
    }

    public double fromDegrees(double degrees) {
        return this == DEGREES ? degrees : Math.toRadians(degrees);
    }

    public double fromUnit(UnnormalizedAngleUnit them, double theirs) {
        return them == RADIANS ? fromRadians(theirs) : fromDegrees(theirs);
    }

    public double toRadians(double inOurUnits) {
        return this == RADIANS ? inOurUnits : Math.toRadians(inOurUnits);
    }

    public double toDegrees(double inOurUnits) {
        return this == DEGREES ? inOurUnits : Math.toDegrees(inOurUnits);
    }

    public AngleUnit getNormalized() {
        return this == DEGREES ? AngleUnit.DEGREES : AngleUnit.RADIANS;
    }
}
