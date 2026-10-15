package org.ftcsim.sdk;

/**
 * Thrown by hardware calls made from an OpMode's thread after STOP. On a real robot, hardware
 * calls from a stopped OpMode fail too, which is what ends loops that never check opModeIsActive().
 */
public class OpModeStoppedException extends RuntimeException {
    public OpModeStoppedException() {
        super("The OpMode was stopped; hardware can no longer be used from it");
    }
}
