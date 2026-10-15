package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public interface IMU extends HardwareDevice {
    class Parameters {
        public ImuOrientationOnRobot imuOrientationOnRobot;

        public Parameters(ImuOrientationOnRobot imuOrientationOnRobot) {
            this.imuOrientationOnRobot = imuOrientationOnRobot;
        }
    }

    boolean initialize(Parameters parameters);

    void resetYaw();

    YawPitchRollAngles getRobotYawPitchRollAngles();

    AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit);
}
