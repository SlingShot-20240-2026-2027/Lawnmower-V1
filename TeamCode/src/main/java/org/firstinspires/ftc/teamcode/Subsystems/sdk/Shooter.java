package org.firstinspires.ftc.teamcode.Subsystems.sdk;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Shooter {

    public static String LEFT_MOTOR_NAME = "shooterleft";
    public static String RIGHT_MOTOR_NAME = "shooterright";
    public static String HOOD_SERVO_NAME = "hood1";
    public static boolean LEFT_REVERSED = true;
    public static boolean RIGHT_REVERSED = false;
    public static double TICKS_PER_REV = 28;
    public static double MAX_RPM = 6000;

    public static double TARGET_RPM = 3000;
    public static double HOOD_POSITION = 0.5;
    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = 1.0;

    public static double kP = 1.2;
    public static double kI = 0.12;
    public static double kD = 0.0;
    public static double kF = 11.7;

    public static boolean VOLTAGE_COMPENSATION = true;
    public static double NOMINAL_VOLTAGE = 12.0;
    public static long VOLTAGE_PERIOD_MS = 200;

    public static double READY_TOLERANCE_RPM = 50;
    public static long READY_HOLD_MS = 80;
    public static long SPINUP_TIMEOUT_MS = 2500;

    public enum Mode {ON, IDLE}

    private final DcMotorEx left;
    private final DcMotorEx right;
    private final DcMotorEx[] motors;
    private final Servo hood;
    private final VoltageSensor voltageSensor;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private Mode mode = Mode.IDLE;
    private double rpmOverride = Double.NaN;
    private double hoodOverride = Double.NaN;

    private boolean velocityModeActive = false;
    private double commandedTicksPerSec = Double.NaN;
    private double appliedP = Double.NaN, appliedI = Double.NaN, appliedD = Double.NaN, appliedF = Double.NaN;

    private double rpm = 0, leftRpm = 0, rightRpm = 0, targetRpm = 0, error = 0;
    private double lastHood = Double.NaN;
    private double voltage;
    private boolean ready = false;

    public Shooter(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, LEFT_MOTOR_NAME);
        right = hardwareMap.get(DcMotorEx.class, RIGHT_MOTOR_NAME);
        motors = new DcMotorEx[]{left, right};
        hood = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        left.setDirection(LEFT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(RIGHT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0);
        }
        voltage = voltageSensor.getVoltage();
    }

    public Action on() {
        return spinUp(Double.NaN, Double.NaN);
    }

    public Action on(double rpm, double hoodPosition) {
        return spinUp(rpm, hoodPosition);
    }

    public Action idle() {
        return Actions.instant(this::setIdle);
    }

    public Action waitUntilReady(long timeoutMs) {
        return Actions.waitUntil(this::isReady, timeoutMs);
    }

    private Action spinUp(double rpm, double hoodPosition) {
        ElapsedTime timer = new ElapsedTime();
        return new Action() {
            public void start() {
                setOn(rpm, hoodPosition);
                timer.reset();
            }

            public boolean update() {
                return ready || timer.milliseconds() >= SPINUP_TIMEOUT_MS;
            }

            public void stop() {
            }
        };
    }

    public void setOn() {
        setOn(Double.NaN, Double.NaN);
    }

    public void setOn(double rpm, double hoodPosition) {
        rpmOverride = rpm;
        hoodOverride = hoodPosition;
        mode = Mode.ON;
        resetReady();
    }

    public void setIdle() {
        if (mode != Mode.IDLE) {
            mode = Mode.IDLE;
            resetReady();
        }
    }

    public void update() {
        if (voltageTimer.milliseconds() >= VOLTAGE_PERIOD_MS) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        leftRpm = toRpm(left.getVelocity());
        rightRpm = toRpm(right.getVelocity());
        rpm = (leftRpm + rightRpm) / 2.0;

        double wanted = mode == Mode.ON ? (Double.isNaN(rpmOverride) ? TARGET_RPM : rpmOverride) : 0.0;
        targetRpm = Range.clip(wanted, 0.0, MAX_RPM);
        error = targetRpm - rpm;

        if (targetRpm > 0) {
            runVelocity(toTicksPerSec(targetRpm));
        } else {
            coast();
        }

        writeHood(Double.isNaN(hoodOverride) ? HOOD_POSITION : hoodOverride);
        updateReady();
    }

    public void stop() {
        mode = Mode.IDLE;
        targetRpm = 0;
        ready = false;
        coast();
    }

    private void runVelocity(double ticksPerSec) {
        if (!velocityModeActive) {
            for (DcMotorEx m : motors) m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            velocityModeActive = true;
            appliedP = appliedI = appliedD = appliedF = Double.NaN;
            commandedTicksPerSec = Double.NaN;
        }
        applyCoefficientsIfChanged();
        if (Double.isNaN(commandedTicksPerSec) || Math.abs(ticksPerSec - commandedTicksPerSec) > 0.5) {
            for (DcMotorEx m : motors) m.setVelocity(ticksPerSec);
            commandedTicksPerSec = ticksPerSec;
        }
    }

    private void coast() {
        if (velocityModeActive || Double.isNaN(commandedTicksPerSec)) {
            for (DcMotorEx m : motors) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(0);
            }
            velocityModeActive = false;
            commandedTicksPerSec = 0;
        }
    }

    private void applyCoefficientsIfChanged() {
        double scale = VOLTAGE_COMPENSATION ? NOMINAL_VOLTAGE / Math.max(voltage, 6.0) : 1.0;
        double f = kF * scale;
        boolean changed = kP != appliedP || kI != appliedI || kD != appliedD
                || Double.isNaN(appliedF) || Math.abs(f - appliedF) > Math.abs(appliedF) * 0.01;
        if (!changed) return;
        for (DcMotorEx m : motors) m.setVelocityPIDFCoefficients(kP, kI, kD, f);
        appliedP = kP;
        appliedI = kI;
        appliedD = kD;
        appliedF = f;
    }

    private void resetReady() {
        readyTimer.reset();
        ready = false;
    }

    private void updateReady() {
        boolean inBand = targetRpm > 0 && Math.abs(error) <= READY_TOLERANCE_RPM;
        if (!inBand) {
            readyTimer.reset();
            ready = false;
        } else {
            ready = readyTimer.milliseconds() >= READY_HOLD_MS;
        }
    }

    private void writeHood(double position) {
        double clipped = Range.clip(position, HOOD_MIN, HOOD_MAX);
        if (Double.isNaN(lastHood) || Math.abs(clipped - lastHood) > 0.0005) {
            hood.setPosition(clipped);
            lastHood = clipped;
        }
    }

    private static double toRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    private static double toTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    public boolean isReady() {
        return ready;
    }

    public Mode getMode() {
        return mode;
    }

    public double getRpm() {
        return rpm;
    }

    public double getLeftRpm() {
        return leftRpm;
    }

    public double getRightRpm() {
        return rightRpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getError() {
        return error;
    }

    public double getVoltage() {
        return voltage;
    }

    public double getHood() {
        return lastHood;
    }

    public double getAppliedF() {
        return Double.isNaN(appliedF) ? 0 : appliedF;
    }

    public double getCurrentAmps() {
        return left.getCurrent(CurrentUnit.AMPS) + right.getCurrent(CurrentUnit.AMPS);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter mode", mode);
        telemetry.addData("Shooter ready", ready);
        telemetry.addData("Shooter rpm (target / avg / L / R)", "%.0f / %.0f / %.0f / %.0f",
                targetRpm, rpm, leftRpm, rightRpm);
        telemetry.addData("Shooter PIDF", "%.3f / %.3f / %.3f / %.3f", kP, kI, kD, getAppliedF());
        telemetry.addData("Shooter hood", "%.3f", lastHood);
        telemetry.addData("Battery voltage", "%.2f", voltage);
    }
}
