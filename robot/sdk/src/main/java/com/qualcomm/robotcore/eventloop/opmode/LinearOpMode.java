package com.qualcomm.robotcore.eventloop.opmode;

import org.ftcsim.sdk.SimBus;

/** An OpMode written as one straight-line method, {@link #runOpMode()}. */
public abstract class LinearOpMode extends OpMode {

    public abstract void runOpMode() throws InterruptedException;

    /** Blocks until START is pressed (or STOP, which ends INIT without starting). */
    public void waitForStart() {
        while (!isStarted() && !isStopRequested()) {
            try {
                SimBus.parkUntilNextTick();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    public final void idle() {
        Thread.yield();
    }

    public final void sleep(long milliseconds) {
        try {
            SimBus.sleepUntil(SimBus.now() + milliseconds / 1000.0);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /** True after START until STOP. Each call also takes one bus slot, like a loop's overhead on a real robot. */
    public final boolean opModeIsActive() {
        boolean active = !isStopRequested() && isStarted();
        if (active) SimBus.transaction(1);
        return active;
    }

    public final boolean opModeInInit() {
        boolean inInit = !isStarted() && !isStopRequested();
        if (inInit) SimBus.transaction(1);
        return inInit;
    }

    public final boolean isStarted() {
        return started || Thread.currentThread().isInterrupted();
    }

    public final boolean isStopRequested() {
        return stopRequested || Thread.currentThread().isInterrupted();
    }

    @Override
    public final void init() {
    }

    @Override
    public final void init_loop() {
    }

    @Override
    public final void start() {
    }

    @Override
    public final void loop() {
    }

    @Override
    public final void stop() {
    }
}
