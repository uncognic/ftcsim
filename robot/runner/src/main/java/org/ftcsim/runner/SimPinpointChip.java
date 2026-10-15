package org.ftcsim.runner;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.ftcsim.sdk.SimBus;

/**
 * The goBILDA Pinpoint odometry computer, emulated at its I2C register interface. The simulator
 * supplies raw pod encoder counts and the chip's gyro; this class does what the chip's firmware
 * does with them: apply encoder directions, resolution and pod offsets, and integrate a field
 * pose. Any driver that speaks the Pinpoint's registers works against it.
 */
final class SimPinpointChip implements LynxI2cDeviceSynch {
    private static final int DEVICE_ID = 1, DEVICE_VERSION = 2, DEVICE_STATUS = 3, DEVICE_CONTROL = 4, LOOP_TIME = 5;
    private static final int X_ENCODER_VALUE = 6, Y_ENCODER_VALUE = 7, X_POSITION = 8, Y_POSITION = 9, H_ORIENTATION = 10;
    private static final int X_VELOCITY = 11, Y_VELOCITY = 12, H_VELOCITY = 13, MM_PER_TICK = 14;
    private static final int X_POD_OFFSET = 15, Y_POD_OFFSET = 16, YAW_SCALAR = 17, BULK_READ = 18;

    private static final int STATUS_READY = 1, STATUS_CALIBRATING = 1 << 1;
    /** The chip holds still while its IMU calibrates after power-up or a reset. */
    private static final double CALIBRATION_SECONDS = 0.25;
    /** Microseconds per chip loop (about 1.5 kHz). */
    private static final int LOOP_TIME_US = 667;

    private final String name;
    private final int bus;
    private I2cAddr address = I2cAddr.create7bit(0x31);
    private String userName;
    private BusSpeed busSpeed = BusSpeed.STANDARD_100K;

    // Configuration registers, written by the driver.
    private float ticksPerMm = 13.26291192f;
    private float xPodOffsetMm;
    private float yPodOffsetMm;
    private float yawScalar = 1f;
    private boolean xReversed;
    private boolean yReversed;

    // Output registers.
    private int status = STATUS_CALIBRATING;
    private int xEncoder, yEncoder;
    private float x, y, heading, vx, vy, vh;

    // Previous raw sample.
    private boolean haveSample;
    private int lastRawX, lastRawY;
    private double lastGyro, lastTime;
    private double calibratedAt = CALIBRATION_SECONDS;

    SimPinpointChip(String name, int bus) {
        this.name = name;
        this.bus = bus;
        this.userName = name;
    }

    /** A new sample from the simulator: raw pod counts and the chip's gyro heading (radians, unwrapped). */
    synchronized void onSample(int rawX, int rawY, double gyroHeading, double gyroRate, double time) {
        if (!haveSample) {
            haveSample = true;
            lastRawX = rawX;
            lastRawY = rawY;
            lastGyro = gyroHeading;
            lastTime = time;
            calibratedAt = time + CALIBRATION_SECONDS;
            return;
        }
        double dt = time - lastTime;
        int dxRaw = rawX - lastRawX, dyRaw = rawY - lastRawY;
        double dGyro = gyroHeading - lastGyro;
        lastRawX = rawX;
        lastRawY = rawY;
        lastGyro = gyroHeading;
        lastTime = time;

        int xSign = xReversed ? -1 : 1, ySign = yReversed ? -1 : 1;
        xEncoder += dxRaw * xSign;
        yEncoder += dyRaw * ySign;
        if (time < calibratedAt) {
            status = STATUS_CALIBRATING;
            vx = vy = vh = 0;
            return;
        }
        status = STATUS_READY;

        double dh = dGyro * yawScalar;
        // A pod off the tracking center also rolls when the robot turns; remove that part.
        double forward = dxRaw * xSign / ticksPerMm + xPodOffsetMm * dh;
        double left = dyRaw * ySign / ticksPerMm - yPodOffsetMm * dh;
        double mid = heading + dh / 2;
        double dx = forward * Math.cos(mid) - left * Math.sin(mid);
        double dy = forward * Math.sin(mid) + left * Math.cos(mid);
        x += dx;
        y += dy;
        heading += dh;
        if (dt > 0) {
            vx = (float) (dx / dt);
            vy = (float) (dy / dt);
        }
        vh = (float) (gyroRate * yawScalar);
    }

    // ---- register file

    // Bus time is paid before taking the chip's lock: paying can park the OpMode thread until the
    // next tick, and the next tick is delivered through onSample(), which needs the lock.

    @Override
    public byte[] read(int ireg, int creg) {
        SimBus.transaction(creg > 8 ? 2 : 1);
        synchronized (this) {
            return readRegister(ireg, creg);
        }
    }

    private byte[] readRegister(int ireg, int creg) {
        ByteBuffer b = ByteBuffer.allocate(Math.max(creg, 4)).order(ByteOrder.LITTLE_ENDIAN);
        switch (ireg) {
            case DEVICE_ID: b.putInt(1); break;
            case DEVICE_VERSION: b.putInt(3); break;
            case DEVICE_STATUS: b.putInt(status); break;
            case LOOP_TIME: b.putInt(LOOP_TIME_US); break;
            case X_ENCODER_VALUE: b.putInt(xEncoder); break;
            case Y_ENCODER_VALUE: b.putInt(yEncoder); break;
            case X_POSITION: b.putFloat(x); break;
            case Y_POSITION: b.putFloat(y); break;
            case H_ORIENTATION: b.putFloat(heading); break;
            case X_VELOCITY: b.putFloat(vx); break;
            case Y_VELOCITY: b.putFloat(vy); break;
            case H_VELOCITY: b.putFloat(vh); break;
            case MM_PER_TICK: b.putFloat(ticksPerMm); break;
            case X_POD_OFFSET: b.putFloat(xPodOffsetMm); break;
            case Y_POD_OFFSET: b.putFloat(yPodOffsetMm); break;
            case YAW_SCALAR: b.putFloat(yawScalar); break;
            case BULK_READ:
                b = ByteBuffer.allocate(Math.max(creg, 40)).order(ByteOrder.LITTLE_ENDIAN);
                b.putInt(status).putInt(LOOP_TIME_US).putInt(xEncoder).putInt(yEncoder)
                        .putFloat(x).putFloat(y).putFloat(heading).putFloat(vx).putFloat(vy).putFloat(vh);
                break;
            default:
                break;
        }
        byte[] out = new byte[creg];
        System.arraycopy(b.array(), 0, out, 0, creg);
        return out;
    }

    @Override
    public void write(int ireg, byte[] data) {
        SimBus.transaction(1);
        synchronized (this) {
            writeRegister(ireg, data);
        }
    }

    private void writeRegister(int ireg, byte[] data) {
        if (data.length < 4) return;
        ByteBuffer b = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
        switch (ireg) {
            case DEVICE_CONTROL: control(b.getInt()); break;
            case X_POSITION: x = b.getFloat(); break;
            case Y_POSITION: y = b.getFloat(); break;
            case H_ORIENTATION: heading = b.getFloat(); break;
            case MM_PER_TICK: ticksPerMm = b.getFloat(); break;
            case X_POD_OFFSET: xPodOffsetMm = b.getFloat(); break;
            case Y_POD_OFFSET: yPodOffsetMm = b.getFloat(); break;
            case YAW_SCALAR: yawScalar = b.getFloat(); break;
            default: break;
        }
    }

    private void control(int bits) {
        if ((bits & 1) != 0) calibratedAt = lastTime + CALIBRATION_SECONDS;
        if ((bits & (1 << 1)) != 0) {
            x = y = heading = 0;
            calibratedAt = lastTime + CALIBRATION_SECONDS;
        }
        if ((bits & (1 << 2)) != 0) yReversed = true;
        if ((bits & (1 << 3)) != 0) yReversed = false;
        if ((bits & (1 << 4)) != 0) xReversed = true;
        if ((bits & (1 << 5)) != 0) xReversed = false;
    }

    // ---- the rest of the I2C client interface

    @Override
    public byte read8(int ireg) {
        return read(ireg, 1)[0];
    }

    @Override
    public void write8(int ireg, int bVal) {
        write(ireg, new byte[] {(byte) bVal});
    }

    @Override
    public void write8(int ireg, int bVal, I2cWaitControl waitControl) {
        write8(ireg, bVal);
    }

    @Override
    public void write(int ireg, byte[] data, I2cWaitControl waitControl) {
        write(ireg, data);
    }

    @Override
    public void waitForWriteCompletions(I2cWaitControl waitControl) {
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        address = newAddress;
    }

    @Override
    public I2cAddr getI2cAddress() {
        return address;
    }

    @Override
    public boolean isArmed() {
        return true;
    }

    @Override
    public void setLogging(boolean enabled) {
    }

    @Override
    public boolean getLogging() {
        return false;
    }

    @Override
    public void setLoggingTag(String loggingTag) {
    }

    @Override
    public String getLoggingTag() {
        return name;
    }

    @Override
    public void setUserConfiguredName(String name) {
        userName = name;
    }

    @Override
    public String getUserConfiguredName() {
        return userName;
    }

    @Override
    public void setBusSpeed(BusSpeed speed) {
        busSpeed = speed;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.GoBilda;
    }

    @Override
    public String getDeviceName() {
        return "goBILDA Pinpoint (simulated)";
    }

    @Override
    public String getConnectionInfo() {
        return "Control Hub; I2C bus " + bus + "; " + address + "; " + busSpeed;
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public void close() {
    }
}
