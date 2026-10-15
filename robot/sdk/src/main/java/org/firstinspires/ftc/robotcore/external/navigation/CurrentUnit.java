package org.firstinspires.ftc.robotcore.external.navigation;

public enum CurrentUnit {
    AMPS, MILLIAMPS;

    public double convert(double value, CurrentUnit fromUnit) {
        if (fromUnit == this) return value;
        return this == AMPS ? value / 1000.0 : value * 1000.0;
    }

    public double toAmps(double current) {
        return AMPS.convert(current, this);
    }

    public double toMilliAmps(double current) {
        return MILLIAMPS.convert(current, this);
    }
}
