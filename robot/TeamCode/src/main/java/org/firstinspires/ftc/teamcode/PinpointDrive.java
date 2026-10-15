package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Drives forward, strafes and turns on timers, and reports where the Pinpoint thinks the robot is. */
@Autonomous(name = "Pinpoint Drive", group = "Examples")
public class PinpointDrive extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); // goBILDA's reference pod placement
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        waitForStart();

        drive(0.5, 0, 0, 1.2);   // forward
        drive(0, 0.5, 0, 1.2);   // strafe right
        drive(0, 0, 0.4, 1.0);   // turn right
        drive(0, 0, 0, 1.0);     // settle

        while (opModeIsActive()) {
            odo.update();
            report();
        }
    }

    private void drive(double y, double x, double rx, double seconds) {
        frontLeft.setPower(y + x + rx);
        backLeft.setPower(y - x + rx);
        frontRight.setPower(y - x - rx);
        backRight.setPower(y + x - rx);
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            odo.update();
            report();
        }
    }

    private void report() {
        Pose2D pos = odo.getPosition();
        telemetry.addData("Status", odo.getDeviceStatus());
        telemetry.addData("Pinpoint X / Y", "%.1f / %.1f in", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH));
        telemetry.addData("Pinpoint heading", "%.1f°", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pods", "%d / %d", odo.getEncoderX(), odo.getEncoderY());
        telemetry.update();
    }
}
