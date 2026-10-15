package com.qualcomm.robotcore.hardware;

public interface HardwareDevice {
    enum Manufacturer {
        Unknown, Other, Lego, HiTechnic, ModernRobotics, Adafruit, Matrix, Lynx, AMS, STMicroelectronics, Broadcom, DFRobot, SparkFun, GoBilda
    }

    Manufacturer getManufacturer();

    String getDeviceName();

    String getConnectionInfo();

    int getVersion();

    void resetDeviceConfigurationForOpMode();

    void close();
}
