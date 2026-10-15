package org.firstinspires.ftc.robotcore.external.navigation;

import java.util.Locale;

public enum DistanceUnit {
    METER(0), CM(1), MM(2), INCH(3);

    public static final double infinity = Double.MAX_VALUE;
    public static final double mmPerInch = 25.4;
    public static final double mPerInch = mmPerInch * 0.001;

    public final byte bVal;

    DistanceUnit(int bVal) {
        this.bVal = (byte) bVal;
    }

    private double metersPerUnit() {
        switch (this) {
            case CM: return 0.01;
            case MM: return 0.001;
            case INCH: return mPerInch;
            default: return 1.0;
        }
    }

    public double fromMeters(double meters) {
        return meters == infinity ? infinity : meters / metersPerUnit();
    }

    public double fromInches(double inches) {
        return fromMeters(INCH.toMeters(inches));
    }

    public double fromCm(double cm) {
        return fromMeters(CM.toMeters(cm));
    }

    public double fromMm(double mm) {
        return fromMeters(MM.toMeters(mm));
    }

    public double fromUnit(DistanceUnit him, double his) {
        return fromMeters(him.toMeters(his));
    }

    public double toMeters(double inOurUnits) {
        return inOurUnits == infinity ? infinity : inOurUnits * metersPerUnit();
    }

    public double toInches(double inOurUnits) {
        return INCH.fromMeters(toMeters(inOurUnits));
    }

    public double toCm(double inOurUnits) {
        return CM.fromMeters(toMeters(inOurUnits));
    }

    public double toMm(double inOurUnits) {
        return MM.fromMeters(toMeters(inOurUnits));
    }

    public String toString(double inOurUnits) {
        return inOurUnits == infinity ? "infinity" : String.format(Locale.getDefault(), "%.3f%s", inOurUnits, toString());
    }

    @Override
    public String toString() {
        switch (this) {
            case METER: return "m";
            case CM: return "cm";
            case MM: return "mm";
            default: return "in";
        }
    }
}
