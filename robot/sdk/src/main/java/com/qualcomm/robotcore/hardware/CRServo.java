package com.qualcomm.robotcore.hardware;

/** A continuous rotation servo, driven with a power like a motor. */
public interface CRServo extends DcMotorSimple {
    int getPortNumber();
}
