package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Drives a 24 inch square: RUN_TO_POSITION for the sides, an IMU P-controller for the turns. */
@Autonomous(name = "Encoder Square", group = "Examples")
public class EncoderSquare extends LinearOpMode {
    static final double TICKS_PER_REV = 537.7;            // goBILDA 312 RPM
    static final double WHEEL_DIAMETER_IN = 104 / 25.4;   // goBILDA 104 mm mecanum
    static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx[] motors;
    private IMU imu;
    private VoltageSensor battery;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        motors = new DcMotorEx[] {frontLeft, frontRight, backLeft, backRight};
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : motors) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        imu.resetYaw();

        for (int side = 1; side <= 4 && opModeIsActive(); side++) {
            driveInches(24, 0.6);
            turnTo(90 * side);
        }
    }

    private void driveInches(double inches, double power) {
        int target = (int) Math.round(inches * TICKS_PER_INCH);
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(target);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 4 && (frontLeft.isBusy() || frontRight.isBusy())) {
            telemetry.addData("Target", target);
            telemetry.addData("Front left / right", "%d / %d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.addData("FL current", "%.2f A", frontLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Battery", "%.2f V", battery.getVoltage());
            telemetry.update();
        }
        for (DcMotorEx m : motors) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /** Turns to an absolute heading in degrees (counterclockwise positive). */
    private void turnTo(double degrees) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 3) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = AngleUnit.normalizeDegrees(degrees - heading);
            if (Math.abs(error) < 1.0) break;
            double turn = Range.clip(error * 0.012, -0.5, 0.5);
            turn += Math.signum(turn) * 0.04;
            // Positive turn rotates counterclockwise: left wheels back, right wheels forward.
            frontLeft.setPower(-turn);
            backLeft.setPower(-turn);
            frontRight.setPower(turn);
            backRight.setPower(turn);
            telemetry.addData("Heading", "%.1f° (target %.0f°)", heading, degrees);
            telemetry.update();
        }
        for (DcMotorEx m : motors) m.setPower(0);
    }
}
