package org.firstinspires.ftc.teamcode.Teleop;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooternf;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

import java.util.List;

@Config
@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTuner extends LinearOpMode {

    public static boolean STEP_TEST = false;
    public static double STEP_LOW_RPM = 2500;
    public static double STEP_HIGH_RPM = 3500;
    public static long STEP_PERIOD_MS = 2000;

    public static boolean FEED_ONLY_WHEN_READY = false;
    public static double TRIGGER_THRESHOLD = 0.3;
    public static double SHOT_DROP_RPM = 100;

    private Shooternf shooter;
    private Intake intake;
    private Transfer transfer;
    private List<LynxModule> hubs;
    private Telemetry dash;

    private final Gamepad g1 = new Gamepad(), g1Prev = new Gamepad();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean stepHigh = false;
    private boolean feeding = false;
    private boolean inDip = false;
    private double dipMinRpm = 0;
    private double lastDropRpm = 0;
    private double lastRecoveryMs = 0;
    private int shots = 0;
    private double loopMs = 0;

    @Override
    public void runOpMode() {
        dash = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dash.setMsTransmissionInterval(50);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Scheduler.reset();
        shooter = new Shooternf(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);

        while (opModeInInit()) {
            dash.addLine("SHOOTER TUNER - edit Shooternf and ShooterTuner in FTC Dashboard > Configuration");
            dash.addLine("A on | B idle | RT hold transfer | Y step test | intake always on");
            dash.addLine("Set RPM and hood in Configuration > Shooternf > TARGET_RPM / HOOD_POSITION");
            dash.update();
            idle();
        }
        if (isStopRequested()) return;

        schedule(shooter.periodic());
        schedule(intake.in(), transfer.idle());
        loopTimer.reset();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            g1Prev.copy(g1);
            g1.copy(gamepad1);

            handleButtons();
            handleStepTest();
            handleFeed();

            Scheduler.execute();

            trackShots();
            report();
        }

        shooter.stop();
        intake.setState(Intake.State.IDLE);
        transfer.setState(Transfer.State.IDLE);
        Scheduler.reset();
    }

    private void handleButtons() {
        if (tapped(g1.a, g1Prev.a)) schedule(shooter.on());
        if (tapped(g1.b, g1Prev.b)) {
            STEP_TEST = false;
            schedule(shooter.idle());
        }

        if (tapped(g1.y, g1Prev.y)) STEP_TEST = !STEP_TEST;
    }

    private void handleStepTest() {
        if (!STEP_TEST) {
            shooter.setTargetRpm(null);
            stepTimer.reset();
            stepHigh = false;
            return;
        }
        if (shooter.getMode() != Shooternf.Mode.ON) schedule(shooter.on());
        if (stepTimer.milliseconds() >= STEP_PERIOD_MS) {
            stepHigh = !stepHigh;
            stepTimer.reset();
        }
        shooter.setTargetRpm(stepHigh ? STEP_HIGH_RPM : STEP_LOW_RPM);
    }

    private void handleFeed() {
        boolean wantTransfer = g1.right_trigger > TRIGGER_THRESHOLD
                && (!FEED_ONLY_WHEN_READY || shooter.isReady());
        if (wantTransfer != feeding) {
            schedule(wantTransfer ? transfer.transfer() : transfer.idle());
            feeding = wantTransfer;
        }
    }

    private void trackShots() {
        double target = shooter.getTargetRpm();
        double rpm = shooter.getRpm();
        if (target <= 0) {
            inDip = false;
            return;
        }
        if (!inDip && feeding && rpm < target - SHOT_DROP_RPM) {
            inDip = true;
            dipMinRpm = rpm;
            shotTimer.reset();
            shots++;
        } else if (inDip) {
            dipMinRpm = Math.min(dipMinRpm, rpm);
            if (shooter.isReady()) {
                inDip = false;
                lastDropRpm = target - dipMinRpm;
                lastRecoveryMs = shotTimer.milliseconds();
            }
        }
    }

    private void report() {
        double ms = loopTimer.milliseconds();
        loopTimer.reset();
        loopMs = loopMs == 0 ? ms : 0.9 * loopMs + 0.1 * ms;

        dash.addData("targetRpm", shooter.getTargetRpm());
        dash.addData("rpm", shooter.getRpm());
        dash.addData("error", shooter.getError());
        dash.addData("power", shooter.getPower());
        dash.addData("ready", shooter.isReady() ? 1 : 0);
        dash.addData("voltage", shooter.getVoltage());

        dash.addLine();
        dash.addData("Mode", "%s%s", shooter.getMode(), STEP_TEST ? "  (STEP TEST)" : "");
        dash.addData("Dashboard TARGET_RPM", "%.0f", Shooternf.TARGET_RPM);
        dash.addData("Dashboard HOOD_POSITION", "%.3f", Shooternf.HOOD_POSITION);
        dash.addData("Hood (actual)", "%.3f", shooter.getHood());
        dash.addData("Gains", "kV %.6f  kS %.3f  kP %.6f  kI %.6f  kD %.6f",
                Shooternf.kV, Shooternf.kS, Shooternf.kP, Shooternf.kI, Shooternf.kD);
        dash.addData("Transfer", feeding ? "RUNNING" : "IDLE");
        dash.addData("Intake", "RUNNING (always on)");
        dash.addData("Shots", shots);
        dash.addData("Last shot drop (rpm)", "%.0f", lastDropRpm);
        dash.addData("Last recovery (ms)", "%.0f", lastRecoveryMs);
        dash.addData("Loop (ms)", "%.1f", loopMs);
        dash.update();
    }

    private static boolean tapped(boolean now, boolean before) {
        return now && !before;
    }
}
