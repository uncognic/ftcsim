package org.firstinspires.ftc.vision.apriltag;

/** One tag seen in one frame. {@code metadata} and {@code ftcPose} are null for tags not in the processor's library. */
public class AprilTagDetection {
    public final int id;
    public final int hamming;
    public final float decisionMargin;
    public final AprilTagMetadata metadata;
    public final AprilTagPoseFtc ftcPose;
    public final long frameAcquisitionNanoTime;

    public AprilTagDetection(int id, int hamming, float decisionMargin, AprilTagMetadata metadata, AprilTagPoseFtc ftcPose, long frameAcquisitionNanoTime) {
        this.id = id;
        this.hamming = hamming;
        this.decisionMargin = decisionMargin;
        this.metadata = metadata;
        this.ftcPose = ftcPose;
        this.frameAcquisitionNanoTime = frameAcquisitionNanoTime;
    }
}
