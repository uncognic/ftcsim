package org.firstinspires.ftc.robotcore.external.navigation;

public enum AngleUnit {
    DEGREES, RADIANS;

    public double fromRadians(double radians) {
        return this == RADIANS ? radians : Math.toDegrees(radians);
    }

    public double fromDegrees(double degrees) {
        return this == DEGREES ? degrees : Math.toRadians(degrees);
    }

    public double fromUnit(AngleUnit them, double theirs) {
        return them == RADIANS ? fromRadians(theirs) : fromDegrees(theirs);
    }

    public double toRadians(double inOurUnits) {
        return this == RADIANS ? inOurUnits : Math.toRadians(inOurUnits);
    }

    public double toDegrees(double inOurUnits) {
        return this == DEGREES ? inOurUnits : Math.toDegrees(inOurUnits);
    }

    /** Wraps an angle in this unit into [-180, 180) degrees or [-pi, pi) radians. */
    public double normalize(double inOurUnits) {
        return this == DEGREES ? normalizeDegrees(inOurUnits) : normalizeRadians(inOurUnits);
    }

    public static double normalizeDegrees(double degrees) {
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    public static double normalizeRadians(double radians) {
        while (radians >= Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
}
