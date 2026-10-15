package com.qualcomm.robotcore.eventloop.opmode;

/** Lifecycle hooks for the runner. Not part of the FTC API; OpModes should not call these. */
public final class OpModeInternals {
    private OpModeInternals() {
    }

    public static void markStarted(OpMode opMode) {
        opMode.resetRuntime();
        opMode.started = true;
    }

    public static void requestStop(OpMode opMode) {
        opMode.stopRequested = true;
    }

    public static boolean wantsToStop(OpMode opMode) {
        return opMode.selfStopRequested;
    }
}
