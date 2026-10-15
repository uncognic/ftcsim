package org.firstinspires.ftc.vision.apriltag;

/**
 * A tag's pose relative to the camera in FTC terms: x right, y forward (out of the lens), z up.
 * Yaw/pitch/roll and bearing/elevation are degrees, distances are in the processor's output unit.
 */
public class AprilTagPoseFtc {
    public final double x;
    public final double y;
    public final double z;
    public final double yaw;
    public final double pitch;
    public final double roll;
    public final double range;
    /** Degrees; positive means the tag is to the left of the camera's center line. */
    public final double bearing;
    public final double elevation;

    public AprilTagPoseFtc(double x, double y, double z, double yaw, double pitch, double roll, double range, double bearing, double elevation) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
        this.range = range;
        this.bearing = bearing;
        this.elevation = elevation;
    }
}
