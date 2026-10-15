package com.qualcomm.hardware.gobilda;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * goBILDA Pinpoint odometry computer. Talks to the device over I2C exactly like the real
 * driver: all state lives on the device and is fetched with a 40-byte bulk read in {@link #update()}.
 * Positions are millimeters and radians on the device.
 */
@I2cDeviceType
@DeviceProperties(name = "goBILDA® Pinpoint Odometry Computer", xmlTag = "goBILDAPinpoint",
        description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)")
public class GoBildaPinpointDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    public static final byte DEFAULT_ADDRESS = 0x31;

    /** Encoder ticks per millimeter of travel for goBILDA's pods. */
    private static final float SWINGARM_POD_TICKS_PER_MM = 13.26291192f;
    private static final float FOUR_BAR_POD_TICKS_PER_MM = 19.89436789f;

    private enum Register {
        DEVICE_ID(1), DEVICE_VERSION(2), DEVICE_STATUS(3), DEVICE_CONTROL(4), LOOP_TIME(5),
        X_ENCODER_VALUE(6), Y_ENCODER_VALUE(7), X_POSITION(8), Y_POSITION(9), H_ORIENTATION(10),
        X_VELOCITY(11), Y_VELOCITY(12), H_VELOCITY(13), MM_PER_TICK(14), X_POD_OFFSET(15),
        Y_POD_OFFSET(16), YAW_SCALAR(17), BULK_READ(18);

        final int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    public enum DeviceStatus {
        NOT_READY(0),
        READY(1),
        CALIBRATING(1 << 1),
        FAULT_X_POD_NOT_DETECTED(1 << 2),
        FAULT_Y_POD_NOT_DETECTED(1 << 3),
        FAULT_NO_PODS_DETECTED(1 << 2 | 1 << 3),
        FAULT_IMU_RUNAWAY(1 << 4),
        FAULT_BAD_READ(1 << 5);

        private final int status;

        DeviceStatus(int status) {
            this.status = status;
        }
    }

    public enum EncoderDirection { FORWARD, REVERSED }

    public enum GoBildaOdometryPods { goBILDA_SWINGARM_POD, goBILDA_4_BAR_POD }

    public enum ReadData { ONLY_UPDATE_HEADING }

    private int deviceStatus;
    private int loopTime;
    private int xEncoderValue;
    private int yEncoderValue;
    private float xPosition;
    private float yPosition;
    private float hOrientation;
    private float xVelocity;
    private float yVelocity;
    private float hVelocity;

    public GoBildaPinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.GoBilda;
    }

    @Override
    public String getDeviceName() {
        return "goBILDA® Pinpoint Odometry Computer";
    }

    // ---- register access (little-endian on the wire)

    private void writeInt(Register reg, int value) {
        deviceClient.write(reg.bVal, ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(value).array());
    }

    private void writeFloat(Register reg, float value) {
        deviceClient.write(reg.bVal, ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(value).array());
    }

    private int readInt(Register reg) {
        return ByteBuffer.wrap(deviceClient.read(reg.bVal, 4)).order(ByteOrder.LITTLE_ENDIAN).getInt();
    }

    private float readFloat(Register reg) {
        return ByteBuffer.wrap(deviceClient.read(reg.bVal, 4)).order(ByteOrder.LITTLE_ENDIAN).getFloat();
    }

    // ---- reading

    /** Reads position, velocity, encoders and status from the device in one transaction. */
    public void update() {
        ByteBuffer b = ByteBuffer.wrap(deviceClient.read(Register.BULK_READ.bVal, 40)).order(ByteOrder.LITTLE_ENDIAN);
        deviceStatus = b.getInt();
        loopTime = b.getInt();
        xEncoderValue = b.getInt();
        yEncoderValue = b.getInt();
        xPosition = b.getFloat();
        yPosition = b.getFloat();
        hOrientation = b.getFloat();
        xVelocity = b.getFloat();
        yVelocity = b.getFloat();
        hVelocity = b.getFloat();
    }

    public void update(ReadData data) {
        if (data == ReadData.ONLY_UPDATE_HEADING) hOrientation = readFloat(Register.H_ORIENTATION);
    }

    // ---- configuration

    /** Where the pods are relative to the tracking center: X pod sideways offset (left +), Y pod forward offset (forward +). */
    public void setOffsets(double xOffset, double yOffset, DistanceUnit distanceUnit) {
        writeFloat(Register.X_POD_OFFSET, (float) distanceUnit.toMm(xOffset));
        writeFloat(Register.Y_POD_OFFSET, (float) distanceUnit.toMm(yOffset));
    }

    public void recalibrateIMU() {
        writeInt(Register.DEVICE_CONTROL, 1);
    }

    public void resetPosAndIMU() {
        writeInt(Register.DEVICE_CONTROL, 1 << 1);
    }

    public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder) {
        writeInt(Register.DEVICE_CONTROL, xEncoder == EncoderDirection.FORWARD ? 1 << 5 : 1 << 4);
        writeInt(Register.DEVICE_CONTROL, yEncoder == EncoderDirection.FORWARD ? 1 << 3 : 1 << 2);
    }

    public void setEncoderResolution(GoBildaOdometryPods pods) {
        writeFloat(Register.MM_PER_TICK, pods == GoBildaOdometryPods.goBILDA_SWINGARM_POD ? SWINGARM_POD_TICKS_PER_MM : FOUR_BAR_POD_TICKS_PER_MM);
    }

    public void setEncoderResolution(double ticksPerUnit, DistanceUnit distanceUnit) {
        writeFloat(Register.MM_PER_TICK, (float) (ticksPerUnit / distanceUnit.toMm(1.0)));
    }

    public void setYawScalar(double yawScalar) {
        writeFloat(Register.YAW_SCALAR, (float) yawScalar);
    }

    public void setPosition(Pose2D pos) {
        setPosX(pos.getX(DistanceUnit.MM), DistanceUnit.MM);
        setPosY(pos.getY(DistanceUnit.MM), DistanceUnit.MM);
        setHeading(pos.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
    }

    public void setPosX(double posX, DistanceUnit distanceUnit) {
        writeFloat(Register.X_POSITION, (float) distanceUnit.toMm(posX));
    }

    public void setPosY(double posY, DistanceUnit distanceUnit) {
        writeFloat(Register.Y_POSITION, (float) distanceUnit.toMm(posY));
    }

    public void setHeading(double heading, AngleUnit angleUnit) {
        writeFloat(Register.H_ORIENTATION, (float) angleUnit.toRadians(heading));
    }

    // ---- getters (values from the last update())

    public int getDeviceID() {
        return readInt(Register.DEVICE_ID);
    }

    public int getDeviceVersion() {
        return readInt(Register.DEVICE_VERSION);
    }

    public float getYawScalar() {
        return readFloat(Register.YAW_SCALAR);
    }

    public DeviceStatus getDeviceStatus() {
        int s = deviceStatus;
        if ((s & DeviceStatus.CALIBRATING.status) != 0) return DeviceStatus.CALIBRATING;
        boolean xMissing = (s & DeviceStatus.FAULT_X_POD_NOT_DETECTED.status) != 0;
        boolean yMissing = (s & DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status) != 0;
        if (xMissing && yMissing) return DeviceStatus.FAULT_NO_PODS_DETECTED;
        if (xMissing) return DeviceStatus.FAULT_X_POD_NOT_DETECTED;
        if (yMissing) return DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
        if ((s & DeviceStatus.FAULT_IMU_RUNAWAY.status) != 0) return DeviceStatus.FAULT_IMU_RUNAWAY;
        if ((s & DeviceStatus.FAULT_BAD_READ.status) != 0) return DeviceStatus.FAULT_BAD_READ;
        return (s & DeviceStatus.READY.status) != 0 ? DeviceStatus.READY : DeviceStatus.NOT_READY;
    }

    /** Microseconds per device loop. */
    public int getLoopTime() {
        return loopTime;
    }

    public double getFrequency() {
        return loopTime != 0 ? 1_000_000.0 / loopTime : 0;
    }

    public int getEncoderX() {
        return xEncoderValue;
    }

    public int getEncoderY() {
        return yEncoderValue;
    }

    public double getPosX(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(xPosition);
    }

    public double getPosY(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(yPosition);
    }

    public double getHeading(AngleUnit angleUnit) {
        return angleUnit.fromRadians(AngleUnit.normalizeRadians(hOrientation));
    }

    public double getHeading(UnnormalizedAngleUnit unnormalizedAngleUnit) {
        return unnormalizedAngleUnit.fromRadians(hOrientation);
    }

    public double getVelX(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(xVelocity);
    }

    public double getVelY(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(yVelocity);
    }

    public double getHeadingVelocity(UnnormalizedAngleUnit unnormalizedAngleUnit) {
        return unnormalizedAngleUnit.fromRadians(hVelocity);
    }

    public float getXOffset(DistanceUnit distanceUnit) {
        return (float) distanceUnit.fromMm(readFloat(Register.X_POD_OFFSET));
    }

    public float getYOffset(DistanceUnit distanceUnit) {
        return (float) distanceUnit.fromMm(readFloat(Register.Y_POD_OFFSET));
    }

    public Pose2D getPosition() {
        return new Pose2D(DistanceUnit.MM, xPosition, yPosition, AngleUnit.RADIANS, AngleUnit.normalizeRadians(hOrientation));
    }
}
