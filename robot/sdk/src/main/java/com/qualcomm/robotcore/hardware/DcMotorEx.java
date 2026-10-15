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

    int getTargetPositionTolerance();
}
