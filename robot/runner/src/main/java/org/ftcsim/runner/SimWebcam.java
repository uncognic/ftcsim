package org.ftcsim.runner;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/** A configured USB webcam. Frames come from the simulator's camera model, not from pixels. */
final class SimWebcam implements WebcamName {
    private final String name;

    SimWebcam(String name) {
        this.name = name;
    }

    @Override
    public boolean isWebcam() {
        return true;
    }

    @Override
    public boolean isCameraDirection() {
        return false;
    }

    @Override
    public boolean isSwitchable() {
        return false;
    }

    @Override
    public boolean isUnknown() {
        return false;
    }

    @Override
    public boolean isAttached() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Simulated webcam " + name;
    }

    @Override
    public String getConnectionInfo() {
        return "Control Hub; USB";
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public void close() {
    }

    @Override
    public String toString() {
        return name;
    }
}
