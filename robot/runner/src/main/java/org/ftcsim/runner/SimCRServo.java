package org.ftcsim.runner;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import java.io.DataOutputStream;
import java.io.IOException;
import org.ftcsim.sdk.SimBus;

/** A continuous rotation servo port. */
final class SimCRServo implements CRServo {
    private final String name;
    private final int port;
    private final SimHardware hardware;

    private volatile Direction direction;
    private volatile double power;
    private volatile boolean pwmEnabled;

    SimCRServo(String name, int port, SimHardware hardware) {
        this.name = name;
        this.port = port;
        this.hardware = hardware;
        resetDeviceConfigurationForOpMode();
    }

    void writeCommand(DataOutputStream out) throws IOException {
        out.writeFloat((float) (direction == Direction.REVERSE ? -power : power));
        out.writeByte(pwmEnabled && hardware.outputsEnabled() ? 1 : 0);
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        direction = Direction.FORWARD;
        power = 0;
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
    public void setPower(double power) {
        SimBus.transaction(1);
        this.power = Range.clip(power, -1.0, 1.0);
        pwmEnabled = true;
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Lynx;
    }

    @Override
    public String getDeviceName() {
        return "Simulated continuous rotation servo " + name;
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
