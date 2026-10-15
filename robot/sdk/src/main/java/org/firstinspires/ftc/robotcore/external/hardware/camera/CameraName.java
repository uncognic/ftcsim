package org.firstinspires.ftc.robotcore.external.hardware.camera;

/** Identifies a camera: a configured webcam or a built-in phone camera. */
public interface CameraName {
    boolean isWebcam();

    boolean isCameraDirection();

    boolean isSwitchable();

    boolean isUnknown();
}
