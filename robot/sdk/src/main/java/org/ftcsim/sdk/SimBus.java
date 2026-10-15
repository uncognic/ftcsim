package org.ftcsim.sdk;

/**
 * The simulated hardware bus and clock. Not part of the FTC API; OpModes never use it directly.
 *
 * <p>On a real robot every hardware call is a round trip to the hub that takes a couple of
 * milliseconds, and that is what sets how fast a control loop runs. Here each call costs a
 * "transaction", and the bus allows {@link #TRANSACTIONS_PER_TICK} per simulator tick. When the
 * OpMode thread runs out, it parks until the next tick. That gives realistic loop times, and it
 * keeps the OpMode in lockstep with physics: the runner waits for the OpMode to park before it
 * answers the simulator, so the same code produces the same run every time.
 */
public final class SimBus {
    /** Hardware calls per 5 ms tick, i.e. about 2.5 ms per call, close to a REV hub over RS-485/USB. */
    public static final int TRANSACTIONS_PER_TICK = 2;

    private static final Object lock = new Object();
    private static long tick;
    private static volatile double time;
    private static int used;
    private static boolean parked;
    private static volatile boolean attached;
    private static Thread userThread;

    private SimBus() {
    }

    /** Simulator time in seconds. */
    public static double now() {
        return time;
    }

    /** Simulator time in nanoseconds, or the system clock if no simulator is attached. */
    public static long nanoTime() {
        return attached ? (long) (time * 1e9) : System.nanoTime();
    }

    /** One hardware call by the current thread. Only the OpMode thread is charged. */
    public static void transaction(int cost) {
        if (!attached || Thread.currentThread() != userThread) return;
        synchronized (lock) {
            try {
                while (used + cost > TRANSACTIONS_PER_TICK && used > 0) parkLocked();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
            used += cost;
        }
    }

    /** Blocks until the simulator advances one tick. */
    public static void parkUntilNextTick() throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) throw new InterruptedException();
        synchronized (lock) {
            parkLocked();
        }
    }

    /** Blocks until simulator time reaches {@code seconds}. */
    public static void sleepUntil(double seconds) throws InterruptedException {
        while (now() < seconds) parkUntilNextTick();
    }

    private static void parkLocked() throws InterruptedException {
        long t = tick;
        parked = true;
        lock.notifyAll();
        while (tick == t) lock.wait();
    }

    // ---- runner side ----

    /** Sets the thread whose hardware calls are metered (the OpMode thread), or null when none runs. */
    public static void setUserThread(Thread thread) {
        synchronized (lock) {
            userThread = thread;
            parked = false;
            lock.notifyAll();
        }
    }

    /** Clears the metered thread if it is still {@code thread}. */
    public static void clearUserThread(Thread thread) {
        synchronized (lock) {
            if (userThread == thread) {
                userThread = null;
                lock.notifyAll();
            }
        }
    }

    /** A new simulator state arrived: start a tick and release the OpMode thread. */
    public static void onTick(double simTime) {
        synchronized (lock) {
            attached = true;
            time = simTime;
            tick++;
            used = 0;
            parked = false;
            lock.notifyAll();
        }
    }

    /**
     * Waits until the OpMode thread has finished its work for this tick (it parked, or no OpMode
     * is running). Returns false if it is still busy after {@code timeoutMs}, which happens when
     * a loop never touches hardware.
     */
    public static boolean awaitParked(long timeoutMs) throws InterruptedException {
        long deadline = System.nanoTime() + timeoutMs * 1_000_000L;
        synchronized (lock) {
            while (!parked && userThread != null && userThread.isAlive()) {
                long remaining = (deadline - System.nanoTime()) / 1_000_000L;
                if (remaining <= 0) return false;
                lock.wait(Math.min(remaining, 1));
            }
            return true;
        }
    }
}
