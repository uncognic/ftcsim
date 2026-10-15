package org.firstinspires.ftc.robotcore.external.navigation;

import java.util.Locale;

/** A point in space with its distance unit. */
public class Position {
    public DistanceUnit unit;
    public double x;
    public double y;
    public double z;
    public long acquisitionTime;

    public Position() {
        this(DistanceUnit.MM, 0, 0, 0, 0);
    }

    public Position(DistanceUnit unit, double x, double y, double z, long acquisitionTime) {
        this.unit = unit;
        this.x = x;
        this.y = y;
        this.z = z;
        this.acquisitionTime = acquisitionTime;
    }

    public Position toUnit(DistanceUnit distanceUnit) {
        return distanceUnit == unit ? this : new Position(distanceUnit,
                distanceUnit.fromUnit(unit, x), distanceUnit.fromUnit(unit, y), distanceUnit.fromUnit(unit, z), acquisitionTime);
    }

    @Override
    public String toString() {
        return String.format(Locale.getDefault(), "(%.3f %.3f %.3f)%s", x, y, z, unit);
    }
}
