package org.firstinspires.ftc.robotcore.external.navigation;

import java.util.Locale;

/** A position on the field and a heading, each stored in the unit it was created with. */
public class Pose2D {
    protected final double x;
    protected final double y;
    protected final DistanceUnit distanceUnit;
    protected final double heading;
    protected final AngleUnit headingUnit;

    public Pose2D(DistanceUnit distanceUnit, double x, double y, AngleUnit headingUnit, double heading) {
        this.distanceUnit = distanceUnit;
        this.x = x;
        this.y = y;
        this.headingUnit = headingUnit;
        this.heading = heading;
    }

    public double getX(DistanceUnit unit) {
        return unit.fromUnit(distanceUnit, x);
    }

    public double getY(DistanceUnit unit) {
        return unit.fromUnit(distanceUnit, y);
    }

    public double getHeading(AngleUnit unit) {
        return unit.fromUnit(headingUnit, heading);
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "(%.3f %.3f)%s  %.3f%s", x, y, distanceUnit, heading,
                headingUnit == AngleUnit.DEGREES ? "°" : "rad");
    }
}
