package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.MathHelpers;

@Config
public class Turret {

    public static final String LEFT_SERVO_NAME = "turretLeft";
    public static final String RIGHT_SERVO_NAME = "turretRight";
    public static final String ENCODER_NAME = "turretEncoder";

    public static double turretOffsetRad = 0;
    public static double MIN_TURRET_ANGLE = Math.toRadians(-157);
    public static double MAX_TURRET_ANGLE = Math.toRadians(157);

    public static double ENCODER_ZERO_DEG = 0;

    public static boolean ENCODER_REVERSED = false;

    public static double ENCODER_MAX_VOLTS = 3.2;

    public static boolean LEFT_REVERSED = false;
    public static boolean RIGHT_REVERSED = false;

    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.05;
    public static double kS = 0.05;
    public static double maxPower = 1.0;
    public static double deadbandRad = Math.toRadians(0.5);
    public static double integralMax = 0.2;

    private final CRServo servoL;
    private final CRServo servoR;
    private final AnalogInput encoder;

    private double targetAngle = 0;
    private double integral = 0;
    private double lastAngle = Double.NaN;
    private double lastTime = Double.NaN;
    private double lastPowerL = Double.NaN, lastPowerR = Double.NaN;

    public Turret(HardwareMap hardwareMap) {
        servoL = hardwareMap.get(CRServo.class, LEFT_SERVO_NAME);
        servoR = hardwareMap.get(CRServo.class, RIGHT_SERVO_NAME);
        servoL.setDirection(LEFT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        servoR.setDirection(RIGHT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        encoder = hardwareMap.get(AnalogInput.class, ENCODER_NAME);
    }

    public double getRawEncoderDegrees() {
        return encoder.getVoltage() / ENCODER_MAX_VOLTS * 360.0;
    }

    public double getRawEncoderVoltage() {
        return encoder.getVoltage();
    }

    public double getCurrentAngle() {
        double angle = Math.toRadians(getRawEncoderDegrees() - ENCODER_ZERO_DEG);
        if (ENCODER_REVERSED) angle = -angle;
        return MathHelpers.wrapAngleRadians(angle);
    }

    public void setTurretAngle(double angleRad) {
        targetAngle = Range.clip(
                MathHelpers.wrapAngleRadians(angleRad + turretOffsetRad),
                MIN_TURRET_ANGLE,
                MAX_TURRET_ANGLE
        );
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getTargetPosition() {
        return targetAngle;
    }

    public boolean atTarget(double toleranceRad) {
        return Math.abs(targetAngle - getCurrentAngle()) < toleranceRad;
    }

    public void update() {
        double now = System.nanoTime() / 1e9;
        double dt = Double.isNaN(lastTime) ? 0.02 : Range.clip(now - lastTime, 1e-3, 0.1);
        lastTime = now;

        double current = getCurrentAngle();
        double vel = Double.isNaN(lastAngle) ? 0 : MathHelpers.wrapAngleRadians(current - lastAngle) / dt;
        lastAngle = current;

        double error = targetAngle - current;

        double power;
        if (Math.abs(error) < deadbandRad) {
            power = 0;
            integral = 0;
        } else {
            integral = (kI == 0) ? 0 : Range.clip(integral + error * dt, -integralMax / kI, integralMax / kI);
            power = kP * error + kI * integral - kD * vel + kS * Math.signum(error);
        }
        power = Range.clip(power, -maxPower, maxPower);

        if (current >= MAX_TURRET_ANGLE && power > 0) power = 0;
        if (current <= MIN_TURRET_ANGLE && power < 0) power = 0;

        setRawPower(power, power);
    }

    public void setRawPower(double left, double right) {
        if (Double.isNaN(lastPowerL) || Math.abs(left - lastPowerL) > 0.01 || (left == 0 && lastPowerL != 0)) {
            servoL.setPower(left);
            lastPowerL = left;
        }
        if (Double.isNaN(lastPowerR) || Math.abs(right - lastPowerR) > 0.01 || (right == 0 && lastPowerR != 0)) {
            servoR.setPower(right);
            lastPowerR = right;
        }
    }

    public void stop() {
        integral = 0;
        setRawPower(0, 0);
    }
}
