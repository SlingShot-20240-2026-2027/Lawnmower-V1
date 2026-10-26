package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.ButterflyDrive;

import java.util.List;

@TeleOp(name = "ButterflyToggleIMU", group = "TeleOp")
public class FlyingTeleopIMU extends LinearOpMode {

    public static String IMU_NAME = "imu";

    // Just in case for defining the orientation, may be removed
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    public static double DEADBAND = 0.05;
    public static double STRAFE_CORRECTION = 1.1;
    public static double SLOW_MODE_SPEED = 0.35;
    public static int TOGGLE_RUMBLE_MS = 200;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private ButterflyDrive butterfly;
    private List<LynxModule> hubs;

    private boolean imuReady = false;
    private boolean lastX = false;
    private boolean lastBack = false;
    private double headingRad = 0.0;
    private int badImuReads = 0;
    private double flPower, frPower, blPower, brPower;

    @Override
    public void runOpMode() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        initDriveMotors();
        initImu();
        butterfly = new ButterflyDrive(hardwareMap);

        while (opModeInInit()) {
            telemetry.addLine("Ready. Keep the robot still and facing away from the driver.");
            telemetry.addLine("X = mecanum / rhino | BACK = reset heading | RT = slow");
            telemetry.addData("IMU", imuReady ? "initialized" : "FAILED - check config / orientation");
            telemetry.addData("Hub orientation", "logo %s, USB %s", LOGO_DIRECTION, USB_DIRECTION);
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        imu.resetYaw();
        headingRad = 0.0;
        butterfly.setMode(ButterflyDrive.Mode.MECANUM);

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            readHeading();

            handleButtons();
            drive();
            updateTelemetry();
        }

        setWheelPowers(0, 0, 0, 0);
    }

    private void initDriveMotors() {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft   = hardwareMap.get(DcMotorEx.class, "bl");
        backRight  = hardwareMap.get(DcMotorEx.class, "br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void initImu() {
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION));
        imuReady = imu.initialize(parameters);
        imu.resetYaw();
    }

    private void readHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        if (angles.getAcquisitionTime() == 0) {
            badImuReads++;
            return;
        }
        headingRad = angles.getYaw(AngleUnit.RADIANS);
    }

    private void handleButtons() {
        boolean x = gamepad1.x;
        if (x && !lastX) {
            butterfly.setMode(butterfly.isTraction()
                    ? ButterflyDrive.Mode.MECANUM
                    : ButterflyDrive.Mode.TRACTION);
            gamepad1.rumble(TOGGLE_RUMBLE_MS);
        }
        lastX = x;

        boolean back = gamepad1.back;
        if (back && !lastBack) {
            imu.resetYaw();
            headingRad = 0.0;
            gamepad1.rumbleBlips(1);
        }
        lastBack = back;
    }

    private void drive() {
        double forward = deadband(-gamepad1.left_stick_y);
        double strafe  = deadband(gamepad1.left_stick_x);
        double turn    = deadband(gamepad1.right_stick_x);

        if (butterfly.isTraction()) {
            strafe = 0;
        } else {
            double cos = Math.cos(-headingRad);
            double sin = Math.sin(-headingRad);
            double robotStrafe  = strafe * cos - forward * sin;
            double robotForward = strafe * sin + forward * cos;
            strafe  = robotStrafe * STRAFE_CORRECTION;
            forward = robotForward;
        }

        double speed = 1.0 - gamepad1.right_trigger * (1.0 - SLOW_MODE_SPEED);
        setWheelPowers(forward, strafe, turn, speed);
    }

    private void setWheelPowers(double forward, double strafe, double turn, double speed) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        flPower = (forward + strafe + turn) / denominator * speed;
        blPower = (forward - strafe + turn) / denominator * speed;
        frPower = (forward - strafe - turn) / denominator * speed;
        brPower = (forward + strafe - turn) / denominator * speed;

        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);
    }

    private static double deadband(double v) {
        return Math.abs(v) < DEADBAND ? 0.0 : v;
    }

    private void updateTelemetry() {
        telemetry.addData("Wheels", butterfly.isTraction()
                ? "RHINO (robot-centric)" : "MECANUM (field-centric)");
        telemetry.addLine("X = toggle | BACK = reset heading | RT = slow");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(headingRad));
        telemetry.addData("IMU bad reads", badImuReads);
        telemetry.addData("Power FL FR", "%.2f  %.2f", flPower, frPower);
        telemetry.addData("Power BL BR", "%.2f  %.2f", blPower, brPower);
        butterfly.addTelemetry(telemetry);
        telemetry.update();
    }
}
