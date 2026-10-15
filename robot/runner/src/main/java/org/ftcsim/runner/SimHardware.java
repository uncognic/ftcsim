package org.ftcsim.runner;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/** The robot's hardware as described by the simulator's CONFIG message, plus gamepads. */
final class SimHardware {
    final String robotName;
    final HardwareMap hardwareMap = new HardwareMap();
    final List<SimDcMotor> motors = new ArrayList<>();
    final SimVoltageSensor voltageSensor = new SimVoltageSensor();
    final Gamepad gamepad1 = new Gamepad();
    final Gamepad gamepad2 = new Gamepad();
    SimImu imu;

    private volatile boolean outputsEnabled;

    SimHardware(DataInputStream config) throws IOException {
        int version = config.readUnsignedShort();
        if (version != Protocol.VERSION) {
            throw new IOException("simulator speaks protocol v" + version + ", runner speaks v" + Protocol.VERSION);
        }
        robotName = Protocol.readString(config);
        int count = config.readUnsignedByte();
        for (int i = 0; i < count; i++) {
            int kind = config.readUnsignedByte();
            String name = Protocol.readString(config);
            int port = config.readUnsignedByte();
            float ticksPerRev = config.readFloat();
            float maxRpm = config.readFloat();
            if (kind == Protocol.DEVICE_MOTOR) {
                SimDcMotor motor = new SimDcMotor(name, port, ticksPerRev, maxRpm, this);
                motors.add(motor);
                hardwareMap.put(name, motor);
            } else if (kind == Protocol.DEVICE_IMU) {
                imu = new SimImu(name);
                hardwareMap.put(name, imu);
            }
        }
        hardwareMap.put("Control Hub", voltageSensor);
    }

    boolean outputsEnabled() {
        return outputsEnabled;
    }

    /** Called at OpMode INIT: the SDK resets every device to its defaults. */
    void resetForOpMode() {
        for (HardwareDevice device : hardwareMap) device.resetDeviceConfigurationForOpMode();
        outputsEnabled = true;
    }

    /** Called at STOP: motors go to zero power regardless of what the OpMode does next. */
    void disableOutputs() {
        outputsEnabled = false;
    }

    /** Applies a STATE message body (after the seq and time fields) to the devices. */
    void applyState(DataInputStream in) throws IOException {
        voltageSensor.voltage = in.readFloat();
        int count = in.readUnsignedByte();
        for (int i = 0; i < count; i++) {
            int position = in.readInt();
            float velocity = in.readFloat();
            float current = in.readFloat();
            if (i < motors.size()) {
                SimDcMotor m = motors.get(i);
                m.rawPosition = position;
                m.rawVelocity = velocity;
                m.current = current;
            }
        }
        float yaw = in.readFloat(), pitch = in.readFloat(), roll = in.readFloat();
        float rx = in.readFloat(), ry = in.readFloat(), rz = in.readFloat();
        if (imu != null) {
            imu.yaw = yaw;
            imu.pitch = pitch;
            imu.roll = roll;
            imu.rateX = rx;
            imu.rateY = ry;
            imu.rateZ = rz;
        }
        readGamepad(in, gamepad1);
        readGamepad(in, gamepad2);
    }

    void writeMotorCommands(DataOutputStream out) throws IOException {
        out.writeByte(motors.size());
        for (SimDcMotor m : motors) m.writeCommand(out);
    }

    private static void readGamepad(DataInputStream in, Gamepad target) throws IOException {
        Gamepad g = new Gamepad();
        g.left_stick_x = in.readFloat();
        g.left_stick_y = in.readFloat();
        g.right_stick_x = in.readFloat();
        g.right_stick_y = in.readFloat();
        g.left_trigger = in.readFloat();
        g.right_trigger = in.readFloat();
        int b = in.readInt();
        g.a = (b & Protocol.A) != 0;
        g.b = (b & Protocol.B) != 0;
        g.x = (b & Protocol.X) != 0;
        g.y = (b & Protocol.Y) != 0;
        g.left_bumper = (b & Protocol.LEFT_BUMPER) != 0;
        g.right_bumper = (b & Protocol.RIGHT_BUMPER) != 0;
        g.dpad_up = (b & Protocol.DPAD_UP) != 0;
        g.dpad_down = (b & Protocol.DPAD_DOWN) != 0;
        g.dpad_left = (b & Protocol.DPAD_LEFT) != 0;
        g.dpad_right = (b & Protocol.DPAD_RIGHT) != 0;
        g.start = (b & Protocol.START) != 0;
        g.back = (b & Protocol.BACK) != 0;
        g.guide = (b & Protocol.GUIDE) != 0;
        g.left_stick_button = (b & Protocol.LEFT_STICK_BUTTON) != 0;
        g.right_stick_button = (b & Protocol.RIGHT_STICK_BUTTON) != 0;
        target.copy(g);
    }
}
