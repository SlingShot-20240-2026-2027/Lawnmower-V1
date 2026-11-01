package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Auton.pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.ButterflyDrive;
import org.firstinspires.ftc.teamcode.Subsystems.DeployRamp;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.LookUpTable;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;

import java.util.List;

@TeleOp(name = "ButterflyToggleIMU", group = "TeleOp")
public class FlyingTeleopIMU extends LinearOpMode {

    public static String IMU_NAME = "imu";
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    public static double DEADBAND = 0.05;
    public static double STRAFE_CORRECTION = 1.1;
    public static double SLOW_MODE_SPEED = 0.35;
    public static int TOGGLE_RUMBLE_MS = 200;

    public static double RED_START_X = 88;
    public static double RED_START_Y = 8;
    public static double RED_START_HEADING_DEG = 90;
    public static double BLUE_START_X = 56;
    public static double BLUE_START_Y = 8;
    public static double BLUE_START_HEADING_DEG = 90;

    public static double TRIGGER_THRESHOLD = 0.3;
    public static double READY_TOLERANCE_RPM = 75;
    public static double SHOT_RPM_DROP = 150;
    public static double RPM_TRIM_STEP = 25;
    public static double TURRET_TRIM_STEP_DEG = 1;
    public static long RELOCALIZE_HOLD_MS = 500;
    public static long VOLTAGE_PERIOD_MS = 250;
    public static double LUT_MIN_DIST_IN = 42.45;
    public static double LUT_MAX_DIST_IN = 155.2;
    public static double LUT_SAFE_MAX_DIST_IN = 999;
    public static int START_POLLEN = 0;

    public enum Alliance {RED, BLUE}

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private PinpointLocalizer localizer;
    private VoltageSensor voltageSensor;
    private List<LynxModule> hubs;

    private ButterflyDrive butterfly;
    private Intake intake;
    private Transfer transfer;
    private Gate gate;
    private DeployRamp ramp;
    private LookUpTable lut;
    private Shooter shooter;
    private Turret turret;

    private final Gamepad g1 = new Gamepad(), g1Prev = new Gamepad();

    private Alliance alliance = Alliance.RED;
    private boolean imuReady = false;
    private boolean slowMode = false;
    private boolean autoAim = true;
    private boolean flywheelOn = false;
    private boolean rampDeployed = false;
    private boolean shotArmed = false;
    private boolean wasReady = false;
    private boolean wasFull = false;
    private boolean relocalizeDone = false;
    private Gate.State gateState = null;
    private int ledState = -1;
    private int badImuReads = 0;
    private int localizerErrors = 0;
    private int shotsFired = 0;

    private double headingRad = 0.0;
    private double poseX, poseY, poseHeadingRad, angularVelRad;
    private double distanceToGoal = 0.0;
    private double targetRpm = 0.0;
    private double voltage = 12.0;
    private double flPower, frPower, blPower, brPower;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime relocalizeTimer = new ElapsedTime();
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
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        butterfly = new ButterflyDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        gate = new Gate(hardwareMap);
        ramp = new DeployRamp(hardwareMap);
        lut = new LookUpTable();
        shooter = new Shooter(hardwareMap, lut);
        turret = new Turret(hardwareMap, 0);

        Alliance poseAlliance = null;
        while (opModeInInit()) {
            g1Prev.copy(g1);
            g1.copy(gamepad1);
            if (tapped(g1.dpad_left, g1Prev.dpad_left)) alliance = Alliance.BLUE;
            if (tapped(g1.dpad_right, g1Prev.dpad_right)) alliance = Alliance.RED;
            if (alliance != poseAlliance) {
                setPoseToAllianceStart();
                poseAlliance = alliance;
            }
            readPose();

            telemetry.addData("ALLIANCE", "%s   (D-pad LEFT = BLUE, RIGHT = RED)", alliance);
            telemetry.addLine("Place the robot on the start spot, facing away from you. Keep it still.");
            telemetry.addData("Start pose", "x %.1f  y %.1f  heading %.0f deg",
                    startX(), startY(), startHeadingDeg());
            telemetry.addData("Pinpoint pose", "x %.1f  y %.1f  heading %.1f deg",
                    poseX, poseY, Math.toDegrees(poseHeadingRad));
            telemetry.addData("IMU", imuReady ? "initialized" : "FAILED - check config / orientation");
            telemetry.addData("Pinpoint errors", localizerErrors);
            telemetry.update();
            updateControllerLight();
            idle();
        }
        if (isStopRequested()) return;

        imu.resetYaw();
        headingRad = 0.0;
        setPoseToAllianceStart();
        intake.setPollenCount(START_POLLEN);

        butterfly.setMode(ButterflyDrive.Mode.MECANUM);
        setGate(Gate.State.CLOSED);
        ramp.setState(DeployRamp.State.UP);
        transfer.setState(Transfer.State.IDLE);
        intake.setState(Intake.State.IDLE);
        turret.startTrack();

        voltage = voltageSensor.getVoltage();
        voltageTimer.reset();
        loopTimer.reset();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            g1Prev.copy(g1);
            g1.copy(gamepad1);

            readHeading();
            readPose();
            readVoltage();

            driveButtons();
            drive();
            intakeControls();
            aimAndShoot();
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

    private double startX() {
        return alliance == Alliance.RED ? RED_START_X : BLUE_START_X;
    }

    private double startY() {
        return alliance == Alliance.RED ? RED_START_Y : BLUE_START_Y;
    }

    private double startHeadingDeg() {
        return alliance == Alliance.RED ? RED_START_HEADING_DEG : BLUE_START_HEADING_DEG;
    }

    private void setPoseToAllianceStart() {
        double heading = Math.toRadians(startHeadingDeg());
        try {
            localizer.setPose(new Pose(startX(), startY(), heading));
        } catch (RuntimeException e) {
            localizerErrors++;
        }
        poseX = startX();
        poseY = startY();
        poseHeadingRad = heading;
        angularVelRad = 0;
    }

    private void readHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        if (angles.getAcquisitionTime() == 0) {
            badImuReads++;
            return;
        }
        headingRad = angles.getYaw(AngleUnit.RADIANS);
    }

    private void readPose() {
        try {
            localizer.update();
            Pose p = localizer.pose();
            double x = p.x(), y = p.y(), h = p.heading();
            if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(h)) {
                localizerErrors++;
                return;
            }
            poseX = x;
            poseY = y;
            poseHeadingRad = h;
            double omega = localizer.velocity().omega;
            angularVelRad = Double.isNaN(omega) ? 0 : omega;
        } catch (RuntimeException e) {
            localizerErrors++;
        }
    }

    private void readVoltage() {
        if (voltageTimer.milliseconds() >= VOLTAGE_PERIOD_MS) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
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

        if (tapped(g1.right_stick_button, g1Prev.right_stick_button)) {
            relocalizeTimer.reset();
        }
        if (g1.right_stick_button) {
            if (!relocalizeDone && relocalizeTimer.milliseconds() >= RELOCALIZE_HOLD_MS) {
                setPoseToAllianceStart();
                relocalizeDone = true;
                gamepad1.rumble(400);
            }
        } else {
            relocalizeDone = false;
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
        if (g1.right_trigger > TRIGGER_THRESHOLD) {
            desired = Intake.State.INTAKE;
        } else if (g1.left_trigger > TRIGGER_THRESHOLD) {
            desired = Intake.State.OUTTAKE;
        } else {
            desired = Intake.State.IDLE;
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

    private void aimAndShoot() {
        double c = Math.cos(poseHeadingRad), s = Math.sin(poseHeadingRad);
        double turretX = poseX + RobotConstants.TURRET_X_OFFSET * c - RobotConstants.TURRET_Y_OFFSET * s;
        double turretY = poseY + RobotConstants.TURRET_X_OFFSET * s + RobotConstants.TURRET_Y_OFFSET * c;
        double goalX = alliance == Alliance.RED ? RobotConstants.RED_GOAL_X : RobotConstants.BLUE_GOAL_X;
        double goalY = alliance == Alliance.RED ? RobotConstants.RED_GOAL_Y : RobotConstants.BLUE_GOAL_Y;
        double dx = goalX - turretX;
        double dy = goalY - turretY;
        distanceToGoal = Math.min(Math.hypot(dx, dy), LUT_SAFE_MAX_DIST_IN);

        if (tapped(g1.y, g1Prev.y)) {
            autoAim = !autoAim;
            if (autoAim) {
                turret.startTrack();
            } else {
                turret.stopTrack();
                turret.resetTurret();
            }
            gamepad1.rumbleBlips(autoAim ? 2 : 1);
        }
        if (tapped(g1.dpad_left, g1Prev.dpad_left)) turret.addOffSetDegrees(TURRET_TRIM_STEP_DEG);
        if (tapped(g1.dpad_right, g1Prev.dpad_right)) turret.addOffSetDegrees(-TURRET_TRIM_STEP_DEG);
        if (tapped(g1.dpad_up, g1Prev.dpad_up)) shooter.addOffsetRPM(RPM_TRIM_STEP);
        if (tapped(g1.dpad_down, g1Prev.dpad_down)) shooter.addOffsetRPM(-RPM_TRIM_STEP);

        turret.update(dx, dy, poseHeadingRad, angularVelRad, voltage);

        if (tapped(g1.left_bumper, g1Prev.left_bumper)) {
            flywheelOn = !flywheelOn;
            gamepad1.rumbleBlips(flywheelOn ? 2 : 1);
        }
        boolean shootHeld = g1.right_bumper;
        boolean spin = flywheelOn || shootHeld;

        if (spin && !shooter.isAutoShooting()) {
            shooter.startTrack();
        } else if (!spin && shooter.isAutoShooting()) {
            shooter.stopTrack();
            shooter.stop();
        }

        targetRpm = spin ? (lut.getRPM(distanceToGoal) + RobotConstants.OFFSET_RPM) * RobotConstants.SPEED_SCALE : 0;
        double rpm = shooter.getCurrentRPM();
        boolean ready = spin && Math.abs(targetRpm - rpm) <= READY_TOLERANCE_RPM;
        boolean feeding = shootHeld && ready;

        if (shootHeld) {
            shooter.isShooting();
        } else {
            shooter.notShooting();
        }

        setGate(shootHeld ? Gate.State.OPEN : Gate.State.CLOSED);
        Transfer.State transferWanted = feeding ? Transfer.State.TRANSFER : Transfer.State.IDLE;
        if (transfer.getState() != transferWanted) transfer.setState(transferWanted);

        shooter.update(distanceToGoal, voltage);
        detectShot(spin, shootHeld);

        if (ready && !wasReady && !shootHeld) gamepad1.rumbleBlips(2);
        wasReady = ready;
    }

    private void setGate(Gate.State wanted) {
        if (gateState != wanted) {
            gate.setState(wanted);
            gateState = wanted;
        }
    }

    private void detectShot(boolean spin, boolean shootHeld) {
        double rpm = shooter.getCurrentRPM();
        if (!spin) {
            shotArmed = false;
        } else if (rpm >= targetRpm - READY_TOLERANCE_RPM) {
            shotArmed = true;
        } else if (shotArmed && shootHeld && rpm < targetRpm - SHOT_RPM_DROP) {
            intake.pollenScored();
            shotsFired++;
            shotArmed = false;
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
        shooter.stopTrack();
        shooter.stop();
        turret.stopTrack();
        turret.stop();
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
        if (wasReady) {
            wanted = 2;
        } else {
            wanted = alliance == Alliance.RED ? 0 : 1;
        }
        if (wanted == ledState) return;
        ledState = wanted;
        if (wanted == 2) {
            gamepad1.setLedColor(0.0, 1.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (wanted == 0) {
            gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    private void updateTelemetry() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        loopMsFiltered = loopMsFiltered == 0 ? loopMs : 0.9 * loopMsFiltered + 0.1 * loopMs;

        boolean inRange = distanceToGoal >= LUT_MIN_DIST_IN && distanceToGoal <= LUT_MAX_DIST_IN;

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Wheels", "%s%s", butterfly.isTraction()
                ? "RHINO (robot-centric)" : "MECANUM (field-centric)", slowMode ? "  SLOW" : "");
        telemetry.addData("Shooter", "%s  %s", shooter.isAutoShooting() ? "ON" : "OFF",
                wasReady ? "READY" : "");
        telemetry.addData("RPM", "%.0f / %.0f  (trim %+.0f)", shooter.getCurrentRPM(), targetRpm,
                RobotConstants.OFFSET_RPM);
        telemetry.addData("Distance (in)", "%.1f%s", distanceToGoal, inRange ? "" : "  OUT OF TABLE RANGE");
        telemetry.addData("Pollen", "%d / %d%s   shots %d", intake.getPollenCount(), Intake.MAX_POLLEN,
                intake.isFull() ? "  FULL" : "", shotsFired);
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Aim", "%s  turret %.1f -> %.1f  (trim %+.0f)", autoAim ? "AUTO" : "HOME",
                turret.getCurrentAngle(), turret.getTargetAngle(), turret.getOffsetDegrees());
        telemetry.addData("Ramp", rampDeployed ? "DEPLOYED" : "UP");
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f deg  w %.0f deg/s", poseX, poseY,
                Math.toDegrees(poseHeadingRad), Math.toDegrees(angularVelRad));
        telemetry.addData("Drive heading (deg)", "%.1f", Math.toDegrees(headingRad));
        telemetry.addData("Pinpoint errors", localizerErrors);
        telemetry.addData("IMU bad reads", badImuReads);
        telemetry.addData("Battery (V)", "%.2f", voltage);
        telemetry.addData("Loop", "%.1f ms", loopMsFiltered);
        telemetry.update();
    }
}