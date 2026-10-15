package org.firstinspires.ftc.robotcore.external.navigation;

/** Rotation rates in robot axes: x right, y forward, z up (counterclockwise-positive). */
public class AngularVelocity {
    public AngleUnit unit;
    public float xRotationRate;
    public float yRotationRate;
    public float zRotationRate;
    public long acquisitionTime;

    public AngularVelocity() {
        this(AngleUnit.DEGREES, 0, 0, 0, 0);
    }

    public AngularVelocity(AngleUnit unit, float xRotationRate, float yRotationRate, float zRotationRate, long acquisitionTime) {
        this.unit = unit;
        this.xRotationRate = xRotationRate;
        this.yRotationRate = yRotationRate;
        this.zRotationRate = zRotationRate;
        this.acquisitionTime = acquisitionTime;
    }

    public AngularVelocity toAngleUnit(AngleUnit target) {
        return new AngularVelocity(target, (float) target.fromUnit(unit, xRotationRate),
                (float) target.fromUnit(unit, yRotationRate), (float) target.fromUnit(unit, zRotationRate), acquisitionTime);
    }
}
