package org.ftcsim.runner;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.io.DataOutputStream;
import java.io.IOException;
import org.ftcsim.sdk.SimBus;

/** A servo port. Direction and scaleRange are applied here, on the SDK side, as on a real robot. */
final class SimServo implements Servo {
    private final String name;
    private final int port;
    private final SimHardware hardware;

    private volatile Direction direction;
    private volatile double scaleMin;
    private volatile double scaleMax;
    private volatile double position;
    private volatile boolean pwmEnabled;

    SimServo(String name, int port, SimHardware hardware) {
        this.name = name;
        this.port = port;
        this.hardware = hardware;
        resetDeviceConfigurationForOpMode();
    }

    void writeCommand(DataOutputStream out) throws IOException {
        double p = direction == Direction.REVERSE ? 1.0 - position : position;
        double scaled = scaleMin + p * (scaleMax - scaleMin);
        out.writeFloat((float) scaled);
        out.writeByte(pwmEnabled && hardware.outputsEnabled() ? 1 : 0);
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        direction = Direction.FORWARD;
        scaleMin = MIN_POSITION;
        scaleMax = MAX_POSITION;
        pwmEnabled = false;
    }

    @Override
    public int getPortNumber() {
        return port;
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
    public void setPosition(double position) {
        SimBus.transaction(1);
        this.position = Range.clip(position, MIN_POSITION, MAX_POSITION);
        pwmEnabled = true;
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public void scaleRange(double min, double max) {
        min = Range.clip(min, MIN_POSITION, MAX_POSITION);
        max = Range.clip(max, MIN_POSITION, MAX_POSITION);
        if (min >= max) throw new IllegalArgumentException("min must be less than max");
        scaleMin = min;
        scaleMax = max;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Lynx;
    }

    @Override
    public String getDeviceName() {
        return "Simulated servo " + name;
    }

    @Override
    public String getConnectionInfo() {
        return "servo port " + port;
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void close() {
        pwmEnabled = false;
    }
}
