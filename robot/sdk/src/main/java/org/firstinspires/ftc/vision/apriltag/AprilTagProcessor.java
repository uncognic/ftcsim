package org.firstinspires.ftc.vision.apriltag;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionProcessor;

/**
 * Finds AprilTags in camera frames. In the simulator, detections come from the simulated
 * camera's view of the field rather than from image processing.
 */
public abstract class AprilTagProcessor implements VisionProcessor {
    public static final int THREADS_DEFAULT = 3;

    public enum TagFamily { TAG_36h11, TAG_25h9, TAG_16h5, TAG_standard41h12 }

    public enum PoseSolver { APRILTAG_BUILTIN, OPENCV_ITERATIVE, OPENCV_SOLVEPNP_EPNP, OPENCV_IPPE, OPENCV_IPPE_SQUARE, OPENCV_SQPNP }

    public static AprilTagProcessor easyCreateWithDefaults() {
        return new Builder().build();
    }

    public static class Builder {
        private AprilTagLibrary library = new AprilTagLibrary.Builder().build();
        private TagFamily tagFamily = TagFamily.TAG_36h11;
        private DistanceUnit outputUnitsLength = DistanceUnit.INCH;
        private AngleUnit outputUnitsAngle = AngleUnit.DEGREES;
        private Position cameraPosition = new Position();
        private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

        public Builder setLensIntrinsics(double fx, double fy, double cx, double cy) {
            return this;
        }

        public Builder setCameraPose(Position position, YawPitchRollAngles ypr) {
            this.cameraPosition = position;
            this.cameraOrientation = ypr;
            return this;
        }

        public Builder setDrawAxes(boolean drawAxes) {
            return this;
        }

        public Builder setDrawCubeProjection(boolean drawCube) {
            return this;
        }

        public Builder setDrawTagOutline(boolean drawOutline) {
            return this;
        }

        public Builder setDrawTagID(boolean drawTagId) {
            return this;
        }

        public Builder setTagFamily(TagFamily tagFamily) {
            this.tagFamily = tagFamily;
            return this;
        }

        public Builder setTagLibrary(AprilTagLibrary tagLibrary) {
            this.library = tagLibrary;
            return this;
        }

        public Builder setOutputUnits(DistanceUnit distanceUnit, AngleUnit angleUnit) {
            this.outputUnitsLength = distanceUnit;
            this.outputUnitsAngle = angleUnit;
            return this;
        }

        public Builder setNumThreads(int threads) {
            return this;
        }

        public AprilTagProcessor build() {
            return new SimAprilTagProcessor(library, tagFamily, outputUnitsLength, outputUnitsAngle, cameraPosition, cameraOrientation);
        }
    }

    /** All detections from the latest frame. */
    public abstract List<AprilTagDetection> getDetections();

    /** Detections from a frame not returned before, or null if no new frame has arrived. */
    public abstract List<AprilTagDetection> getFreshDetections();

    public abstract void setDecimation(float decimation);

    public abstract void setPoseSolver(PoseSolver poseSolver);

    public abstract int getPerTagAvgPoseSolveTime();

    static final class SimAprilTagProcessor extends AprilTagProcessor {
        final AprilTagLibrary library;
        final TagFamily tagFamily;
        final DistanceUnit distanceUnit;
        final AngleUnit angleUnit;
        final Position cameraPosition;
        final YawPitchRollAngles cameraOrientation;
        private volatile List<AprilTagDetection> detections = Collections.emptyList();
        private volatile boolean fresh;

        SimAprilTagProcessor(AprilTagLibrary library, TagFamily tagFamily, DistanceUnit distanceUnit, AngleUnit angleUnit,
                             Position cameraPosition, YawPitchRollAngles cameraOrientation) {
            this.library = library;
            this.tagFamily = tagFamily;
            this.distanceUnit = distanceUnit;
            this.angleUnit = angleUnit;
            this.cameraPosition = cameraPosition;
            this.cameraOrientation = cameraOrientation;
        }

        /** A new frame was processed. */
        void publish(List<AprilTagDetection> frameDetections) {
            detections = Collections.unmodifiableList(new ArrayList<>(frameDetections));
            fresh = true;
        }

        @Override
        public List<AprilTagDetection> getDetections() {
            return detections;
        }

        @Override
        public List<AprilTagDetection> getFreshDetections() {
            if (!fresh) return null;
            fresh = false;
            return detections;
        }

        @Override
        public void setDecimation(float decimation) {
        }

        @Override
        public void setPoseSolver(PoseSolver poseSolver) {
        }

        @Override
        public int getPerTagAvgPoseSolveTime() {
            return 0;
        }
    }
}
