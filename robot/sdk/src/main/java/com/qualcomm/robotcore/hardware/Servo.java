package com.qualcomm.robotcore.hardware;

/** A positional servo. Positions run from 0 to 1 across the servo's range (after {@link #scaleRange}). */
public interface Servo extends HardwareDevice {
    double MIN_POSITION = 0.0;
    double MAX_POSITION = 1.0;

    enum Direction { FORWARD, REVERSE }

    int getPortNumber();

    void setDirection(Direction direction);

    Direction getDirection();

    void setPosition(double position);

    /** The last commanded position. Servos have no feedback; this is not where the servo actually is. */
    double getPosition();

    void scaleRange(double min, double max);
}
