package org.firstinspires.ftc.vision.apriltag;

import org.opencv.core.Point;

/** One tag seen in one frame. {@code metadata} and {@code ftcPose} are null for tags not in the processor's library. */
public class AprilTagDetection {
    public final int id;
    public final int hamming;
    public final float decisionMargin;
    /** Tag center in image pixels. */
    public final Point center;
    /** Tag corners in image pixels, counterclockwise from the bottom left. */
    public final Point[] corners;
    public final AprilTagMetadata metadata;
    public final AprilTagPoseFtc ftcPose;
    public final long frameAcquisitionNanoTime;

    public AprilTagDetection(int id, int hamming, float decisionMargin, Point center, Point[] corners,
                             AprilTagMetadata metadata, AprilTagPoseFtc ftcPose, long frameAcquisitionNanoTime) {
        this.id = id;
        this.hamming = hamming;
        this.decisionMargin = decisionMargin;
        this.center = center;
        this.corners = corners;
        this.metadata = metadata;
        this.ftcPose = ftcPose;
        this.frameAcquisitionNanoTime = frameAcquisitionNanoTime;
    }
}
