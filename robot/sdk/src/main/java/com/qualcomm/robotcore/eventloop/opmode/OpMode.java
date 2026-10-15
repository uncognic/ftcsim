package com.qualcomm.robotcore.eventloop.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.ftcsim.sdk.SimBus;

/**
 * An iterative OpMode: {@link #init()} once, {@link #init_loop()} until START, {@link #start()}
 * once, then {@link #loop()} every cycle until STOP, then {@link #stop()}.
 */
public abstract class OpMode {
    public Gamepad gamepad1 = new Gamepad();
    public Gamepad gamepad2 = new Gamepad();
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    /** Seconds since the OpMode started; updated before each loop. */
    public double time = 0.0;

    volatile boolean started;
    volatile boolean stopRequested;
    volatile boolean selfStopRequested;
    private double startTime;

    public abstract void init();

    public void init_loop() {
    }

    public void start() {
    }

    public abstract void loop();

    public void stop() {
    }

    public double getRuntime() {
        return SimBus.now() - startTime;
    }

    public void resetRuntime() {
        startTime = SimBus.now();
    }

    /** Asks the Robot Controller to stop this OpMode, as if STOP were pressed. */
    public final void requestOpModeStop() {
        selfStopRequested = true;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.update();
    }
}
