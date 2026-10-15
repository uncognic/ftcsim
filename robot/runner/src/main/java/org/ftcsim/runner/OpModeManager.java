package org.ftcsim.runner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeInternals;
import java.util.List;
import org.ftcsim.sdk.OpModeStoppedException;
import org.ftcsim.sdk.SimBus;

/** Runs one OpMode at a time on its own thread, following INIT / START / STOP from the Driver Station. */
final class OpModeManager {
    enum State {
        IDLE(0), INIT(1), RUNNING(2), STOPPED(3);

        final int code;

        State(int code) {
            this.code = code;
        }
    }

    /** Real SDK behaviour: an OpMode that ignores STOP for this long is reported as stuck. */
    private static final long STOP_TIMEOUT_MS = 1000;

    private final List<OpModeScanner.Entry> opModes;
    private final SimHardware hardware;

    private volatile State state = State.IDLE;
    private volatile String error = "";
    private OpMode active;
    private Thread thread;
    private SimTelemetry telemetry;

    OpModeManager(List<OpModeScanner.Entry> opModes, SimHardware hardware) {
        this.opModes = opModes;
        this.hardware = hardware;
    }

    State state() {
        return state;
    }

    String error() {
        return error;
    }

    SimTelemetry telemetry() {
        return telemetry;
    }

    void init(int index) {
        stop();
        if (index < 0 || index >= opModes.size()) {
            error = "No OpMode with index " + index;
            return;
        }
        OpModeScanner.Entry entry = opModes.get(index);
        OpMode opMode;
        try {
            opMode = entry.type.getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException | RuntimeException e) {
            error = "Could not create " + entry.type.getName() + ": " + e;
            state = State.STOPPED;
            return;
        }
        hardware.resetForOpMode();
        telemetry = new SimTelemetry();
        opMode.hardwareMap = hardware.hardwareMap;
        opMode.telemetry = telemetry;
        opMode.gamepad1 = hardware.gamepad1;
        opMode.gamepad2 = hardware.gamepad2;
        opMode.resetRuntime();

        active = opMode;
        error = "";
        state = State.INIT;
        thread = new Thread(() -> run(opMode), "OpMode " + entry.name);
        thread.setDaemon(true);
        SimBus.setUserThread(thread);
        thread.start();
        System.out.println("[runner] INIT " + entry.name);
    }

    void start() {
        if (state != State.INIT || active == null) return;
        OpModeInternals.markStarted(active);
        state = State.RUNNING;
        System.out.println("[runner] START");
    }

    void stop() {
        if (active == null) return;
        OpModeInternals.requestStop(active);
        hardware.disableOutputs();
        Thread t = thread;
        if (t != null && t.isAlive()) {
            SimBus.markStopped(t);
            t.interrupt();
            try {
                t.join(STOP_TIMEOUT_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            if (t.isAlive()) {
                error = "The OpMode \"" + t.getName().substring("OpMode ".length())
                        + "\" is stuck in stop(). It must exit quickly when STOP is pressed (check loops for opModeIsActive()).";
                SimBus.setUserThread(null);
            }
        }
        active = null;
        thread = null;
        state = State.STOPPED;
        System.out.println("[runner] STOP");
    }

    /** Called by the bus thread every tick; handles OpModes that end themselves. */
    void poll() {
        OpMode m = active;
        if (m != null && OpModeInternals.wantsToStop(m)) stop();
    }

    private void run(OpMode opMode) {
        try {
            if (opMode instanceof LinearOpMode) {
                ((LinearOpMode) opMode).runOpMode();
            } else {
                runIterative(opMode);
            }
        } catch (InterruptedException | OpModeStoppedException e) {
            // STOP pressed while the OpMode was waiting or using hardware: normal exit.
        } catch (Throwable t) {
            if (!(t.getCause() instanceof InterruptedException)) {
                error = describe(t);
                System.err.println("[runner] " + error);
                t.printStackTrace();
            }
        } finally {
            // A thread abandoned by stop() must not touch the state of a newer OpMode.
            if (active == opMode) {
                hardware.disableOutputs();
                state = State.STOPPED;
            }
            SimBus.clearUserThread(Thread.currentThread());
        }
    }

    private void runIterative(OpMode m) throws InterruptedException {
        m.init();
        m.telemetry.update();
        while (state == State.INIT) {
            m.init_loop();
            m.telemetry.update();
            SimBus.parkUntilNextTick();
        }
        if (state != State.RUNNING) return;
        m.start();
        while (state == State.RUNNING && !Thread.currentThread().isInterrupted()) {
            m.time = m.getRuntime();
            m.loop();
            m.telemetry.update();
            SimBus.parkUntilNextTick();
        }
        m.stop();
    }

    /** The Driver Station's "User code threw an uncaught exception" message, with the offending line. */
    static String describe(Throwable t) {
        StringBuilder sb = new StringBuilder("User code threw an uncaught exception: ")
                .append(t.getClass().getSimpleName());
        if (t.getMessage() != null) sb.append(" - ").append(t.getMessage());
        for (StackTraceElement e : t.getStackTrace()) {
            String c = e.getClassName();
            if (c.startsWith("java.") || c.startsWith("jdk.") || c.startsWith("com.qualcomm.") || c.startsWith("org.ftcsim.")
                    || c.startsWith("org.firstinspires.ftc.robotcore.")) {
                continue;
            }
            sb.append(" (at ").append(e.getFileName()).append(':').append(e.getLineNumber()).append(')');
            break;
        }
        return sb.toString();
    }
}
