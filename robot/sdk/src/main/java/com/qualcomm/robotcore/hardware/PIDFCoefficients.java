package com.qualcomm.robotcore.hardware;

import java.util.Locale;

/**
 * Hub motor controller gains. Output is on a 32767 = full power scale and errors are in encoder
 * ticks (position) or ticks per second (velocity), so F is about 32767 / max ticks per second.
 */
public class PIDFCoefficients {
    public double p;
    public double i;
    public double d;
    public double f;
    public MotorControlAlgorithm algorithm;

    public PIDFCoefficients() {
        this(0, 0, 0, 0);
    }

    public PIDFCoefficients(double p, double i, double d, double f) {
        this(p, i, d, f, MotorControlAlgorithm.PIDF);
    }

    public PIDFCoefficients(double p, double i, double d, double f, MotorControlAlgorithm algorithm) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.algorithm = algorithm;
    }

    public PIDFCoefficients(PIDFCoefficients other) {
        this(other.p, other.i, other.d, other.f, other.algorithm);
    }

    @SuppressWarnings("deprecation")
    public PIDFCoefficients(PIDCoefficients other) {
        this(other.p, other.i, other.d, 0, MotorControlAlgorithm.LegacyPID);
    }

    @Override
    public String toString() {
        return String.format(Locale.getDefault(), "%s(p=%f i=%f d=%f f=%f alg=%s)", getClass().getSimpleName(), p, i, d, f, algorithm);
    }
}
