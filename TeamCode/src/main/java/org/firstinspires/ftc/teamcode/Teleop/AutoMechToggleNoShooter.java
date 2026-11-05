package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
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
@TeleOp(name = "AutoMechToggle No Shooter", group = "TeleOp")
public class AutoMechToggleNoShooter extends LinearOpMode {

    public static double DEADBAND = 0.05;
    public static double TRANSLATE_EXPO = 0.35;
    public static double TURN_EXPO = 0.55;
    public static double MAX_TURN = 0.85;
    public static double STRAFE_CORRECTION = 1.1;
    public static double SLOW_MODE_SPEED = 0.35;
    public static double TRANSITION_STRAFE_SCALE = 0.25;
    public static int MODE_RUMBLE_MS = 120;
    public static double TRIGGER_THRESHOLD = 0.3;

    public static double ENTER_TRACTION_CONE_DEG = 15.0;
    public static double EXIT_TRACTION_CONE_DEG = 30.0;
    public static double TRACTION_MIN_POWER = 0.35;
    public static double ENTER_TRACTION_MAX_TURN = 0.15;
    public static double EXIT_TRACTION_MAX_TURN = 0.35;
    public static long ENTER_TRACTION_MS = 150;
    public static long EXIT_TRACTION_MS = 40;

    public static long FEED_CLEAR_MS = 300;
    public static int START_POLLEN = 0;

    private static final String FL = "fl", FR = "fr", BL = "bl", BR = "br";

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private List<LynxModule> hubs;

    private ButterflyDrive butterfly;
    private Intake intake;
    private Transfer transfer;
    private Gate gate;
    private DeployRamp ramp;

    private final Gamepad g1 = new Gamepad(), g1Prev = new Gamepad();

    private boolean autoSwitch = true;
    private boolean slowMode = false;
    private boolean rampDeployed = false;
    private ButterflyDrive.Mode manualMode = ButterflyDrive.Mode.MECANUM;

    private ButterflyDrive.Mode pendingMode = null;
    private long pendingSinceMs = 0;
    private String modeReason = "start";

    private boolean feeding = false;
    private boolean wasFull = false;
    private final ElapsedTime feedTimer = new ElapsedTime();

    private Gate.State gateState = null;
    private int ledState = -1;

    private final ElapsedTime matchClock = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMsFiltered = 0;
    private double pFL, pFR, pBL, pBR;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        initDriveMotors();

        butterfly = new ButterflyDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        gate = new Gate(hardwareMap);
        ramp = new DeployRamp(hardwareMap);

        while (opModeInInit()) {
            telemetry.addLine("Ready. Just drive - wheels switch automatically.");
            telemetry.addLine("RT hold transfer | intake always on | LT reverse intake | X wheels | Back auto");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        intake.setPollenCount(START_POLLEN);
        butterfly.setMode(ButterflyDrive.Mode.MECANUM);
        setGate(Gate.State.CLOSED);
        ramp.setState(DeployRamp.State.UP);
        transfer.setState(Transfer.State.IDLE);
        intake.setState(Intake.State.INTAKE);
        updateControllerLight();
        matchClock.reset();
        loopTimer.reset();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            g1Prev.copy(g1);
            g1.copy(gamepad1);

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
        frontLeft  = hardwareMap.get(DcMotorEx.class, FL);
        frontRight = hardwareMap.get(DcMotorEx.class, FR);
        backLeft   = hardwareMap.get(DcMotorEx.class, BL);
        backRight  = hardwareMap.get(DcMotorEx.class, BR);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void driveButtons() {
        if (tapped(g1.x, g1Prev.x)) {
            ButterflyDrive.Mode current = butterfly.getMode();
            manualMode = current == ButterflyDrive.Mode.MECANUM
                    ? ButterflyDrive.Mode.TRACTION : ButterflyDrive.Mode.MECANUM;
            autoSwitch = false;
            pendingMode = null;
        }

        if (tapped(g1.back, g1Prev.back)) {
            autoSwitch = !autoSwitch;
            manualMode = butterfly.getMode();
            pendingMode = null;
            gamepad1.rumbleBlips(autoSwitch ? 2 : 1);
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
        double rawF = -g1.left_stick_y, rawS = g1.left_stick_x;
        double mag = Math.hypot(rawF, rawS);
        double forward = 0, strafe = 0;
        if (mag > DEADBAND) {
            double scaled = Math.min(1.0, (mag - DEADBAND) / (1.0 - DEADBAND));
            forward = rawF / mag * scaled;
            strafe  = rawS / mag * scaled;
        }
        double turn = rescaleDeadband(g1.right_stick_x);

        ButterflyDrive.Mode target;
        if (!autoSwitch) {
            target = manualMode;
            modeReason = "manual";
        } else {
            target = autoSelectMode(forward, strafe, turn);
        }

        if (target != butterfly.getMode()) {
            butterfly.setMode(target);
            if (MODE_RUMBLE_MS > 0) gamepad1.rumble(MODE_RUMBLE_MS);
        }

        double tMag = Math.hypot(forward, strafe);
        if (tMag > 1e-6) {
            double shaped = expo(Math.min(1.0, tMag), TRANSLATE_EXPO);
            forward = forward / tMag * shaped;
            strafe  = strafe  / tMag * shaped;
        }
        turn = expo(turn, TURN_EXPO) * MAX_TURN;

        if (butterfly.isTraction()) {
            strafe = 0;
        } else {
            strafe *= STRAFE_CORRECTION;
            if (!butterfly.isSettled()) strafe *= TRANSITION_STRAFE_SCALE;
        }

        double speed = slowMode ? SLOW_MODE_SPEED : 1.0;
        setWheelPowers(forward, strafe, turn, speed);
    }

    private ButterflyDrive.Mode autoSelectMode(double forward, double strafe, double turn) {
        ButterflyDrive.Mode current = butterfly.getMode();
        double translateMag = Math.hypot(forward, strafe);
        double absTurn = Math.abs(turn);

        if (translateMag < 1e-3 && absTurn < 1e-3) {
            pendingMode = null;
            modeReason = "idle - holding";
            return current;
        }

        double angle = translateMag < 1e-3 ? 0.0
                : Math.toDegrees(Math.atan2(Math.abs(strafe), Math.abs(forward)));
        ButterflyDrive.Mode wanted;

        if (current == ButterflyDrive.Mode.TRACTION) {
            if (absTurn > EXIT_TRACTION_MAX_TURN) {
                wanted = ButterflyDrive.Mode.MECANUM;
                modeReason = "turning";
            } else if (translateMag >= 1e-3 && angle > EXIT_TRACTION_CONE_DEG) {
                wanted = ButterflyDrive.Mode.MECANUM;
                modeReason = "strafing";
            } else {
                wanted = ButterflyDrive.Mode.TRACTION;
                modeReason = "driving straight";
            }
        } else {
            boolean straight = translateMag >= 1e-3 && angle <= ENTER_TRACTION_CONE_DEG;
            boolean strong = Math.abs(forward) >= TRACTION_MIN_POWER;
            boolean notTurning = absTurn <= ENTER_TRACTION_MAX_TURN;
            if (straight && strong && notTurning) {
                wanted = ButterflyDrive.Mode.TRACTION;
                modeReason = "driving straight";
            } else {
                wanted = ButterflyDrive.Mode.MECANUM;
                modeReason = !notTurning ? "turning" : !straight ? "strafing" : "slow / precise";
            }
        }

        if (wanted == current) {
            pendingMode = null;
            return current;
        }
        long now = (long) matchClock.milliseconds();
        if (pendingMode != wanted) {
            pendingMode = wanted;
            pendingSinceMs = now;
        }
        long required = (wanted == ButterflyDrive.Mode.TRACTION) ? ENTER_TRACTION_MS : EXIT_TRACTION_MS;
        if (now - pendingSinceMs >= required) {
            pendingMode = null;
            return wanted;
        }
        modeReason += " (pending)";
        return current;
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
        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                                            Math.max(Math.abs(fr), Math.abs(br))));
        pFL = fl / max * speed;
        pBL = bl / max * speed;
        pFR = fr / max * speed;
        pBR = br / max * speed;

        frontLeft.setPower(pFL);
        backLeft.setPower(pBL);
        frontRight.setPower(pFR);
        backRight.setPower(pBR);
    }

    private void stopAll() {
        setWheelPowers(0, 0, 0, 0);
        intake.setState(Intake.State.IDLE);
        transfer.setState(Transfer.State.IDLE);
    }

    private static double rescaleDeadband(double v) {
        double a = Math.abs(v);
        if (a <= DEADBAND) return 0.0;
        return Math.signum(v) * Math.min(1.0, (a - DEADBAND) / (1.0 - DEADBAND));
    }

    private static double expo(double v, double e) {
        return (1.0 - e) * v + e * v * v * v;
    }

    private static boolean tapped(boolean now, boolean before) {
        return now && !before;
    }

    private void updateControllerLight() {
        int wanted;
        if (!autoSwitch) {
            wanted = 3;
        } else if (butterfly.isTraction()) {
            wanted = 1;
        } else {
            wanted = 2;
        }
        if (wanted == ledState) return;
        ledState = wanted;
        if (wanted == 1) {
            gamepad1.setLedColor(1.0, 0.45, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (wanted == 2) {
            gamepad1.setLedColor(0.0, 0.4, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0.6, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    private void updateTelemetry() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        loopMsFiltered = loopMsFiltered == 0 ? loopMs : 0.9 * loopMsFiltered + 0.1 * loopMs;

        telemetry.addData("Wheels", "%s  (%s)%s", butterfly.getMode(), modeReason, slowMode ? "  SLOW" : "");
        telemetry.addData("Auto-switch", autoSwitch ? "ON" : "OFF (X = toggle, Back = auto)");
        telemetry.addData("Pollen", "%d / %d%s", intake.getPollenCount(), Intake.MAX_POLLEN,
                intake.isFull() ? "  FULL" : "");
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Transfer", feeding ? "RUNNING (gate open)" : "IDLE");
        telemetry.addData("Ramp", rampDeployed ? "DEPLOYED" : "UP");
        telemetry.addData("Power FL FR", "%.2f  %.2f", pFL, pFR);
        telemetry.addData("Power BL BR", "%.2f  %.2f", pBL, pBR);
        telemetry.addData("Loop", "%.1f ms", loopMsFiltered);
        telemetry.update();
    }
}
