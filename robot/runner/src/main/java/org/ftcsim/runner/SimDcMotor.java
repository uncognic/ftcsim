package org.ftcsim.runner;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import java.io.DataOutputStream;
import java.io.IOException;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.ftcsim.sdk.SimBus;

/**
 * A motor port on the simulated hub. Like the real SDK, direction is applied here (on the
 * SDK side) and closed-loop modes run in the hub firmware, which lives in the simulator.
 */
final class SimDcMotor implements DcMotorEx {
    private final String name;
    private final int port;
    private final double ticksPerRev;
    private final double maxTicksPerSecond;
    private final SimHardware hardware;

    private volatile Direction direction;
    private volatile RunMode mode;
    private volatile ZeroPowerBehavior zeroPowerBehavior;
    private volatile double power;
    private volatile int target;
    private volatile boolean targetSet;
    private volatile int tolerance;
    private volatile boolean enabled;
    private volatile double currentAlert;
    // Hub defaults. Like the real hub, these persist across OpModes until the robot restarts.
    private volatile PIDFCoefficients velocityPidf = new PIDFCoefficients(10, 3, 0, 0);
    private volatile double positionP = 10;

    // Latest readings from the simulator, in the motor's raw direction.
    volatile int rawPosition;
    volatile double rawVelocity;
    volatile double current;

    SimDcMotor(String name, int port, double ticksPerRev, double maxRpm, SimHardware hardware) {
        this.name = name;
        this.port = port;
        this.ticksPerRev = ticksPerRev;
        this.maxTicksPerSecond = maxRpm / 60.0 * ticksPerRev;
        this.hardware = hardware;
        resetDeviceConfigurationForOpMode();
    }

    private int sign() {
        return direction == Direction.REVERSE ? -1 : 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        direction = Direction.FORWARD;
        mode = RunMode.RUN_WITHOUT_ENCODER;
        zeroPowerBehavior = ZeroPowerBehavior.BRAKE;
        power = 0;
        targetSet = false;
        tolerance = 5;
        enabled = true;
        currentAlert = 5.0;
    }

    void writeCommand(DataOutputStream out) throws IOException {
        int modeCode;
        switch (mode) {
            case RUN_USING_ENCODER: modeCode = 1; break;
            case RUN_TO_POSITION: modeCode = 2; break;
            case STOP_AND_RESET_ENCODER: modeCode = 3; break;
            default: modeCode = 0;
        }
        boolean live = enabled && hardware.outputsEnabled();
        out.writeByte(modeCode);
        out.writeByte(zeroPowerBehavior == ZeroPowerBehavior.FLOAT ? 1 : 0);
        out.writeFloat(live ? (float) (power * sign()) : 0f);
        out.writeInt(target * sign());
        PIDFCoefficients v = velocityPidf;
        out.writeFloat((float) v.p);
        out.writeFloat((float) v.i);
        out.writeFloat((float) v.d);
        out.writeFloat((float) v.f);
        out.writeFloat((float) positionP);
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public void setPower(double power) {
        SimBus.transaction(1);
        this.power = Range.clip(power, -1.0, 1.0);
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public int getPortNumber() {
        return port;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        SimBus.transaction(1);
        if (zeroPowerBehavior != ZeroPowerBehavior.UNKNOWN) this.zeroPowerBehavior = zeroPowerBehavior;
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0);
    }

    @Override
    public boolean getPowerFloat() {
        return zeroPowerBehavior == ZeroPowerBehavior.FLOAT && power == 0;
    }

    @Override
    public void setTargetPosition(int position) {
        SimBus.transaction(1);
        target = position;
        targetSet = true;
    }

    @Override
    public int getTargetPosition() {
        return target;
    }

    @Override
    public boolean isBusy() {
        SimBus.transaction(1);
        return mode == RunMode.RUN_TO_POSITION && power != 0 && Math.abs(target - rawPosition * sign()) > tolerance;
    }

    @Override
    public int getCurrentPosition() {
        SimBus.transaction(1);
        return rawPosition * sign();
    }

    @Override
    public void setMode(RunMode mode) {
        mode = mode.migrate();
        if (mode == RunMode.RUN_TO_POSITION && !targetSet) throw new TargetPositionNotSetException();
        SimBus.transaction(1);
        this.mode = mode;
        if (mode == RunMode.STOP_AND_RESET_ENCODER) {
            // The reset happens in hub firmware. Like the real SDK, don't return until it has,
            // so a getCurrentPosition() right after reads 0.
            try {
                SimBus.parkUntilNextTick();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            rawPosition = 0;
        }
    }

    @Override
    public RunMode getMode() {
        return mode;
    }

    @Override
    public void setMotorEnable() {
        enabled = true;
    }

    @Override
    public void setMotorDisable() {
        enabled = false;
    }

    @Override
    public boolean isMotorEnabled() {
        return enabled;
    }

    @Override
    public void setVelocity(double angularRate) {
        setPower(angularRate / maxTicksPerSecond);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        setVelocity(unit.toRadians(angularRate) / (2 * Math.PI) * ticksPerRev);
    }

    @Override
    public double getVelocity() {
        SimBus.transaction(1);
        return rawVelocity * sign();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return unit.fromRadians(getVelocity() / ticksPerRev * 2 * Math.PI);
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        SimBus.transaction(1);
        return unit.convert(current, CurrentUnit.AMPS);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return unit.convert(currentAlert, CurrentUnit.AMPS);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        currentAlert = CurrentUnit.AMPS.convert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        SimBus.transaction(1);
        return current > currentAlert;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        this.tolerance = tolerance;
    }

    @Override
    public int getTargetPositionTolerance() {
        return tolerance;
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) {
        SimBus.transaction(1);
        mode = mode.migrate();
        if (mode == RunMode.RUN_USING_ENCODER) {
            velocityPidf = new PIDFCoefficients(pidfCoefficients);
        } else if (mode == RunMode.RUN_TO_POSITION) {
            positionP = pidfCoefficients.p;
        } else {
            throw new IllegalArgumentException("PIDF coefficients can only be set for RUN_USING_ENCODER or RUN_TO_POSITION, not " + mode);
        }
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, 0, 0, 0));
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        mode = mode.migrate();
        if (mode == RunMode.RUN_TO_POSITION) return new PIDFCoefficients(positionP, 0, 0, 0);
        return new PIDFCoefficients(velocityPidf);
    }

    @Override
    @SuppressWarnings("deprecation")
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        setPIDFCoefficients(mode, new PIDFCoefficients(pidCoefficients));
    }

    @Override
    @SuppressWarnings("deprecation")
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        PIDFCoefficients c = getPIDFCoefficients(mode);
        return new PIDCoefficients(c.p, c.i, c.d);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Lynx;
    }

    @Override
    public String getDeviceName() {
        return "Simulated motor " + name;
    }

    @Override
    public String getConnectionInfo() {
        return "Control Hub; port " + port;
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void close() {
        power = 0;
    }
}
