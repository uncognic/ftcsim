package org.firstinspires.ftc.robotcore.external.hardware.camera;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/** A USB webcam from the robot configuration. */
public interface WebcamName extends CameraName, HardwareDevice {
    boolean isAttached();
}
