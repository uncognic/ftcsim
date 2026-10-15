package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public interface DcMotorEx extends DcMotor {
    void setMotorEnable();

    void setMotorDisable();

    boolean isMotorEnabled();

    /** Target velocity in encoder ticks per second (RUN_USING_ENCODER). */
    void setVelocity(double angularRate);

    void setVelocity(double angularRate, AngleUnit unit);

    /** Encoder ticks per second. */
    double getVelocity();

    double getVelocity(AngleUnit unit);

    double getCurrent(CurrentUnit unit);

    double getCurrentAlert(CurrentUnit unit);

    void setCurrentAlert(double current, CurrentUnit unit);

    boolean isOverCurrent();

    void setTargetPositionTolerance(int tolerance);

    /** Sets the hub's gains for RUN_USING_ENCODER (velocity) or RUN_TO_POSITION (position, P only). */
    void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException;

    void setVelocityPIDFCoefficients(double p, double i, double d, double f);

    void setPositionPIDFCoefficients(double p);

    PIDFCoefficients getPIDFCoefficients(RunMode mode);

    @Deprecated
    void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients);

    @Deprecated
    PIDCoefficients getPIDCoefficients(RunMode mode);

    int getTargetPositionTolerance();
}
