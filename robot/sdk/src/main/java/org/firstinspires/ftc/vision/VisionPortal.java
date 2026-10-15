package org.firstinspires.ftc.vision;

import android.util.Size;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

/** Opens a camera and feeds its frames to vision processors. */
public abstract class VisionPortal {
    public static final int DEFAULT_VIEW_CONTAINER_ID = 0;

    public enum StreamFormat { YUY2, MJPEG }

    public enum CameraState {
        OPENING_CAMERA_DEVICE, CAMERA_DEVICE_READY, STARTING_STREAM, STREAMING,
        STOPPING_STREAM, CLOSING_CAMERA_DEVICE, CAMERA_DEVICE_CLOSED, ERROR
    }

    public static class Builder {
        private CameraName camera;
        private BuiltinCameraDirection builtinDirection;
        private Size resolution = new Size(640, 480);
        private StreamFormat streamFormat = StreamFormat.YUY2;
        private final List<VisionProcessor> processors = new ArrayList<>();

        public Builder setCamera(CameraName camera) {
            this.camera = camera;
            return this;
        }

        public Builder setCamera(BuiltinCameraDirection cameraDirection) {
            this.builtinDirection = cameraDirection;
            return this;
        }

        public Builder setCameraResolution(Size cameraResolution) {
            this.resolution = cameraResolution;
            return this;
        }

        public Builder setStreamFormat(StreamFormat streamFormat) {
            this.streamFormat = streamFormat;
            return this;
        }

        public Builder enableLiveView(boolean enableLiveView) {
            return this;
        }

        public Builder setAutoStopLiveView(boolean autoPause) {
            return this;
        }

        public Builder setLiveViewContainerId(int liveViewContainerId) {
            return this;
        }

        public Builder setShowStatsOverlay(boolean showStatsOverlay) {
            return this;
        }

        public Builder addProcessor(VisionProcessor processor) {
            processors.add(processor);
            return this;
        }

        public Builder addProcessors(VisionProcessor... processors) {
            Collections.addAll(this.processors, processors);
            return this;
        }

        public VisionPortal build() {
            if (camera == null && builtinDirection == null) {
                throw new RuntimeException("You must call setCamera() on the VisionPortal.Builder before build()");
            }
            if (builtinDirection != null && camera == null) {
                throw new RuntimeException("The simulated robot has no phone, so built-in camera "
                        + builtinDirection + " cannot be opened. Configure a webcam and use setCamera(hardwareMap.get(WebcamName.class, ...)).");
            }
            return new SimVisionPortal(camera, resolution, streamFormat, processors);
        }
    }

    public static VisionPortal easyCreateWithDefaults(CameraName cameraName, VisionProcessor... processors) {
        return new Builder().setCamera(cameraName).addProcessors(processors).build();
    }

    public static VisionPortal easyCreateWithDefaults(BuiltinCameraDirection cameraDirection, VisionProcessor... processors) {
        return new Builder().setCamera(cameraDirection).addProcessors(processors).build();
    }

    public abstract void setProcessorEnabled(VisionProcessor processor, boolean enabled);

    public abstract boolean getProcessorEnabled(VisionProcessor processor);

    public abstract CameraState getCameraState();

    public abstract void stopStreaming();

    public abstract void resumeStreaming();

    public abstract void stopLiveView();

    public abstract void resumeLiveView();

    public abstract float getFps();

    public abstract void close();

    /** The simulator's portal. Frames come from the simulated camera, not from pixels. */
    static final class SimVisionPortal extends VisionPortal {
        private final CameraName camera;
        private final Size resolution;
        private final List<VisionProcessor> processors;
        private final List<VisionProcessor> disabled = new ArrayList<>();
        private volatile CameraState state = CameraState.STREAMING;

        SimVisionPortal(CameraName camera, Size resolution, StreamFormat format, List<VisionProcessor> processors) {
            this.camera = camera;
            this.resolution = resolution;
            this.processors = new ArrayList<>(processors);
        }

        @Override
        public synchronized void setProcessorEnabled(VisionProcessor processor, boolean enabled) {
            if (enabled) disabled.remove(processor);
            else if (!disabled.contains(processor)) disabled.add(processor);
        }

        @Override
        public synchronized boolean getProcessorEnabled(VisionProcessor processor) {
            return processors.contains(processor) && !disabled.contains(processor);
        }

        @Override
        public CameraState getCameraState() {
            return state;
        }

        @Override
        public void stopStreaming() {
            state = CameraState.CAMERA_DEVICE_READY;
        }

        @Override
        public void resumeStreaming() {
            state = CameraState.STREAMING;
        }

        @Override
        public void stopLiveView() {
        }

        @Override
        public void resumeLiveView() {
        }

        @Override
        public float getFps() {
            return state == CameraState.STREAMING ? 30f : 0f;
        }

        @Override
        public void close() {
            state = CameraState.CAMERA_DEVICE_CLOSED;
        }

        CameraName camera() {
            return camera;
        }

        Size resolution() {
            return resolution;
        }
    }
}
