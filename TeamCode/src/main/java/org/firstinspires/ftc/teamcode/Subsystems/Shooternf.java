package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Shooternf {

    public static String LEFT_MOTOR_NAME = "shooterleft";
    public static String RIGHT_MOTOR_NAME = "shooterright";
    public static String ENCODER_MOTOR_NAME = "shooterleft";
    public static String HOOD_SERVO_NAME = "hood1";
    public static boolean LEFT_REVERSED = true;
    public static boolean RIGHT_REVERSED = false;
    public static double ENCODER_SIGN = 1;
    public static double TICKS_PER_REV = 28;

    public static double TARGET_RPM = 3000;
    public static double HOOD_POSITION = 0.5;
    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = 1.0;

    public static double kP = 0.0005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.00017;
    public static double kS = 0.0;

    public static double I_ZONE_RPM = 300;
    public static double INTEGRAL_MAX_POWER = 0.25;
    public static double VELOCITY_FILTER = 1.0;
    public static boolean VOLTAGE_COMPENSATION = true;
    public static double NOMINAL_VOLTAGE = 12.0;
    public static long VOLTAGE_PERIOD_MS = 200;
    public static double MIN_POWER = 0.0;
    public static double MAX_POWER = 1.0;

    public static double READY_TOLERANCE_RPM = 50;
    public static long READY_HOLD_MS = 80;

    public enum Mode {ON, IDLE}

    private final DcMotorEx left;
    private final DcMotorEx right;
    private final DcMotorEx encoder;
    private final Servo hood;
    private final VoltageSensor voltageSensor;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private Mode mode = Mode.IDLE;
    private Double rpmOverride = null;
    private Double hoodOverride = null;

    private double rpm = 0;
    private double lastRpm = 0;
    private double targetRpm = 0;
    private double error = 0;
    private double integral = 0;
    private double power = 0;
    private double lastPower = Double.NaN;
    private double lastHood = Double.NaN;
    private double voltage = 12.0;
    private double pTerm, iTerm, dTerm, ffTerm;
    private boolean ready = false;
    private boolean firstUpdate = true;

    public Shooternf(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, LEFT_MOTOR_NAME);
        right = hardwareMap.get(DcMotorEx.class, RIGHT_MOTOR_NAME);
        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_MOTOR_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        left.setDirection(LEFT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(RIGHT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        voltage = voltageSensor.getVoltage();
    }

    public Command on() {
        return instant(() -> {
            rpmOverride = null;
            hoodOverride = null;
            setMode(Mode.ON);
        }).requiring(this);
    }

    public Command on(double rpm, double hoodPosition) {
        return instant(() -> {
            rpmOverride = rpm;
            hoodOverride = hoodPosition;
            setMode(Mode.ON);
        }).requiring(this);
    }

    public Command idle() {
        return instant(() -> setMode(Mode.IDLE)).requiring(this);
    }

    public Command waitUntilReady() {
        return Command.build()
                .setDone(this::isReady);
    }

    public Command waitUntilReady(long timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(timer::reset)
                .setDone(() -> ready || timer.milliseconds() >= timeoutMs);
    }

    public Command periodic() {
        return Command.build()
                .setStart(this::resetController)
                .setExecute(this::update)
                .setEnd(endCondition -> stop());
    }

    public void setMode(Mode newMode) {
        if (newMode != mode) {
            mode = newMode;
            resetController();
        }
    }

    public void setTargetRpm(Double rpmOrNull) {
        rpmOverride = rpmOrNull;
    }

    public void setHood(Double positionOrNull) {
        hoodOverride = positionOrNull;
    }

    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (voltageTimer.milliseconds() >= VOLTAGE_PERIOD_MS) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        double measured = ENCODER_SIGN * encoder.getVelocity() * 60.0 / TICKS_PER_REV;
        double alpha = Range.clip(VELOCITY_FILTER, 0.0, 1.0);
        rpm = firstUpdate ? measured : alpha * measured + (1.0 - alpha) * rpm;

        double newTarget = mode == Mode.ON ? (rpmOverride != null ? rpmOverride : TARGET_RPM) : 0.0;
        if (newTarget != targetRpm) {
            integral = 0;
        }
        targetRpm = Math.max(0.0, newTarget);
        error = targetRpm - rpm;

        if (targetRpm <= 0) {
            pTerm = iTerm = dTerm = ffTerm = 0;
            integral = 0;
            power = 0;
        } else {
            ffTerm = kV * targetRpm + kS;
            pTerm = kP * error;

            if (kI != 0 && Math.abs(error) <= I_ZONE_RPM && dt > 0 && dt < 0.5) {
                integral += error * dt;
                double integralLimit = INTEGRAL_MAX_POWER / Math.abs(kI);
                integral = Range.clip(integral, -integralLimit, integralLimit);
            } else if (kI == 0 || Math.abs(error) > I_ZONE_RPM) {
                integral = 0;
            }
            iTerm = kI * integral;

            dTerm = (!firstUpdate && dt > 0 && dt < 0.5) ? -kD * (rpm - lastRpm) / dt : 0;

            double raw = ffTerm + pTerm + iTerm + dTerm;
            if (VOLTAGE_COMPENSATION) {
                raw *= NOMINAL_VOLTAGE / Math.max(voltage, 6.0);
            }
            power = Range.clip(raw, MIN_POWER, MAX_POWER);
        }

        lastRpm = rpm;
        firstUpdate = false;
        writePower(power);
        writeHood(hoodOverride != null ? hoodOverride : HOOD_POSITION);
        updateReady();
    }

    public void stop() {
        mode = Mode.IDLE;
        targetRpm = 0;
        power = 0;
        integral = 0;
        ready = false;
        writePower(0);
    }

    private void resetController() {
        integral = 0;
        loopTimer.reset();
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

    private void writePower(double p) {
        if (Double.isNaN(lastPower) || Math.abs(p - lastPower) > 0.001 || (p == 0 && lastPower != 0)) {
            left.setPower(p);
            right.setPower(p);
            lastPower = p;
        }
    }

    private void writeHood(double position) {
        double clipped = Range.clip(position, HOOD_MIN, HOOD_MAX);
        if (Double.isNaN(lastHood) || Math.abs(clipped - lastHood) > 0.0005) {
            hood.setPosition(clipped);
            lastHood = clipped;
        }
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

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getError() {
        return error;
    }

    public double getPower() {
        return power;
    }

    public double getVoltage() {
        return voltage;
    }

    public double getHood() {
        return lastHood;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter mode", mode);
        telemetry.addData("Shooter ready", ready);
        telemetry.addData("Shooter target rpm", targetRpm);
        telemetry.addData("Shooter rpm", rpm);
        telemetry.addData("Shooter error", error);
        telemetry.addData("Shooter power", power);
        telemetry.addData("Shooter ff / p / i / d", "%.3f / %.3f / %.3f / %.3f", ffTerm, pTerm, iTerm, dTerm);
        telemetry.addData("Shooter hood", lastHood);
        telemetry.addData("Battery voltage", voltage);
    }
}
