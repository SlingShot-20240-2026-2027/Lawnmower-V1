package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.ButterflyDrive;
import org.firstinspires.ftc.teamcode.Subsystems.DeployRamp;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

import java.util.List;

@Config
@TeleOp(name = "FlyingTeleopIMU No Shooter", group = "TeleOp")
public class FlyingTeleopIMUNoShooter extends LinearOpMode {

    public static String IMU_NAME = "imu";
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    public static double DEADBAND = 0.05;
    public static double STRAFE_CORRECTION = 1.1;
    public static double SLOW_MODE_SPEED = 0.35;
    public static int TOGGLE_RUMBLE_MS = 200;
    public static double TRIGGER_THRESHOLD = 0.3;

    public static long FEED_CLEAR_MS = 300;
    public static int START_POLLEN = 0;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private List<LynxModule> hubs;

    private ButterflyDrive butterfly;
    private Intake intake;
    private Transfer transfer;
    private Gate gate;
    private DeployRamp ramp;

    private final Gamepad g1 = new Gamepad(), g1Prev = new Gamepad();

    private boolean imuReady = false;
    private boolean slowMode = false;
    private boolean rampDeployed = false;

    private boolean feeding = false;
    private boolean wasFull = false;
    private final ElapsedTime feedTimer = new ElapsedTime();

    private Gate.State gateState = null;
    private int ledState = -1;
    private int badImuReads = 0;

    private double headingRad = 0.0;
    private double flPower, frPower, blPower, brPower;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMsFiltered = 0;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        initDriveMotors();
        initImu();

        butterfly = new ButterflyDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        gate = new Gate(hardwareMap);
        ramp = new DeployRamp(hardwareMap);

        while (opModeInInit()) {
            telemetry.addLine("Ready. Point the robot away from you and keep it still.");
            telemetry.addLine("RT hold transfer | intake always on | LT reverse intake | X wheels | Back reset forward");
            telemetry.addData("IMU", imuReady ? "initialized" : "FAILED - check config / orientation");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        imu.resetYaw();
        headingRad = 0.0;
        intake.setPollenCount(START_POLLEN);

        butterfly.setMode(ButterflyDrive.Mode.MECANUM);
        setGate(Gate.State.CLOSED);
        ramp.setState(DeployRamp.State.UP);
        transfer.setState(Transfer.State.IDLE);
        intake.setState(Intake.State.INTAKE);
        updateControllerLight();
        loopTimer.reset();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            g1Prev.copy(g1);
            g1.copy(gamepad1);

            readHeading();

            driveButtons();
            drive();
            intakeControls();
            feedControls();
            intake.update();

            updateControllerLight();
            updateTelemetry();
        }

        stopAll();
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

    private void driveButtons() {
        if (tapped(g1.x, g1Prev.x)) {
            butterfly.setMode(butterfly.isTraction()
                    ? ButterflyDrive.Mode.MECANUM
                    : ButterflyDrive.Mode.TRACTION);
            gamepad1.rumble(TOGGLE_RUMBLE_MS);
        }

        if (tapped(g1.back, g1Prev.back)) {
            imu.resetYaw();
            headingRad = 0.0;
            gamepad1.rumbleBlips(1);
        }

        if (tapped(g1.left_stick_button, g1Prev.left_stick_button)) {
            slowMode = !slowMode;
            gamepad1.rumbleBlips(slowMode ? 2 : 1);
        }

        if (tapped(g1.a, g1Prev.a)) {
            rampDeployed = !rampDeployed;
            ramp.setState(rampDeployed ? DeployRamp.State.DEPLOY : DeployRamp.State.UP);
            gamepad1.rumbleBlips(1);
        }
    }

    private void drive() {
        double forward = deadband(-g1.left_stick_y);
        double strafe  = deadband(g1.left_stick_x);
        double turn    = deadband(g1.right_stick_x);

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

        double speed = slowMode ? SLOW_MODE_SPEED : 1.0;
        setWheelPowers(forward, strafe, turn, speed);
    }

    private void intakeControls() {
        Intake.State desired;
        if (g1.left_trigger > TRIGGER_THRESHOLD) {
            desired = Intake.State.OUTTAKE;
        } else {
            desired = Intake.State.INTAKE;
        }

        Intake.State current = intake.getState();
        boolean ejecting = current == Intake.State.EJECTING;
        if (current != desired && (!ejecting || desired == Intake.State.OUTTAKE)) {
            intake.setState(desired);
        }

        if (tapped(g1.b, g1Prev.b)) {
            intake.resetPollenCount();
            gamepad1.rumbleBlips(1);
        }

        boolean full = intake.isFull();
        if (full && !wasFull) gamepad1.rumbleBlips(3);
        wasFull = full;
    }

    private void feedControls() {
        boolean wantFeed = g1.right_trigger > TRIGGER_THRESHOLD;
        if (wantFeed == feeding) return;

        if (wantFeed) {
            feedTimer.reset();
        } else if (feedTimer.milliseconds() >= FEED_CLEAR_MS) {
            intake.resetPollenCount();
        }
        setGate(wantFeed ? Gate.State.OPEN : Gate.State.CLOSED);
        transfer.setState(wantFeed ? Transfer.State.TRANSFER : Transfer.State.IDLE);
        feeding = wantFeed;
    }

    private void setGate(Gate.State wanted) {
        if (gateState != wanted) {
            gate.setState(wanted);
            gateState = wanted;
        }
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

    private void stopAll() {
        setWheelPowers(0, 0, 0, 0);
        intake.setState(Intake.State.IDLE);
        transfer.setState(Transfer.State.IDLE);
    }

    private static double deadband(double v) {
        return Math.abs(v) < DEADBAND ? 0.0 : v;
    }

    private static boolean tapped(boolean now, boolean before) {
        return now && !before;
    }

    private void updateControllerLight() {
        int wanted;
        if (butterfly.isTraction()) {
            wanted = 1;
        } else {
            wanted = 2;
        }
        if (wanted == ledState) return;
        ledState = wanted;
        if (wanted == 1) {
            gamepad1.setLedColor(1.0, 0.45, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0.0, 0.4, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    private void updateTelemetry() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        loopMsFiltered = loopMsFiltered == 0 ? loopMs : 0.9 * loopMsFiltered + 0.1 * loopMs;

        telemetry.addData("Wheels", "%s%s", butterfly.isTraction()
                ? "RHINO (robot-centric)" : "MECANUM (field-centric)", slowMode ? "  SLOW" : "");
        telemetry.addData("Pollen", "%d / %d%s", intake.getPollenCount(), Intake.MAX_POLLEN,
                intake.isFull() ? "  FULL" : "");
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Transfer", feeding ? "RUNNING (gate open)" : "IDLE");
        telemetry.addData("Ramp", rampDeployed ? "DEPLOYED" : "UP");
        telemetry.addData("Drive heading (deg)", "%.1f", Math.toDegrees(headingRad));
        telemetry.addData("IMU bad reads", badImuReads);
        telemetry.addData("Loop", "%.1f ms", loopMsFiltered);
        telemetry.update();
    }
}
