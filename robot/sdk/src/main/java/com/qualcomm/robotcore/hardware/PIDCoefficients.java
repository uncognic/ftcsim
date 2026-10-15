package com.qualcomm.robotcore.hardware;

import java.util.Locale;

@Deprecated
public class PIDCoefficients {
    public double p;
    public double i;
    public double d;

    public PIDCoefficients() {
    }

    public PIDCoefficients(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    @Override
    public String toString() {
        return String.format(Locale.getDefault(), "%s(p=%f i=%f d=%f)", getClass().getSimpleName(), p, i, d);
    }
}
