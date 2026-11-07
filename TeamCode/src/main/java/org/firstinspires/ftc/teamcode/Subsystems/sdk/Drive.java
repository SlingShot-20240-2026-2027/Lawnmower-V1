package org.firstinspires.ftc.teamcode.Subsystems.sdk;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class Drive {

    public static String FRONT_LEFT_NAME = "fl";
    public static String FRONT_RIGHT_NAME = "fr";
    public static String BACK_LEFT_NAME = "bl";
    public static String BACK_RIGHT_NAME = "br";
    public static String IMU_NAME = "imu";
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public static double FORWARD_IPS_PER_POWER = 50.0;
    public static double STRAFE_IPS_PER_POWER = 40.0;
    public static double MOVE_POWER = 0.5;
    public static long RAMP_MS = 250;
    public static long SETTLE_MS = 150;

    public static double HOLD_KP = 0.02;
    public static double HOLD_MAX = 0.3;

    public static double TURN_KP = 0.02;
    public static double TURN_MAX = 0.5;
    public static double TURN_MIN = 0.1;
    public static double TURN_TOLERANCE_DEG = 2.0;
    public static long TURN_SETTLE_MS = 150;
    public static long TURN_TIMEOUT_MS = 3000;

    public static boolean VOLTAGE_COMPENSATION = true;
    public static double NOMINAL_VOLTAGE = 12.0;

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final DcMotorEx[] motors;
    private final IMU imu;
    private final VoltageSensor voltageSensor;

    private final double startHeadingDeg;
    private double lastHeadingDeg;
    private double voltageScale = 1.0;

    private double estX, estY, estHeadingDeg;
    private String step = "idle";

    public Drive(HardwareMap hardwareMap, double startX, double startY, double startHeadingDeg) {
        frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);
        motors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setPower(0);
        }

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION)));
        imu.resetYaw();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.startHeadingDeg = startHeadingDeg;
        lastHeadingDeg = startHeadingDeg;
        estX = startX;
        estY = startY;
        estHeadingDeg = startHeadingDeg;
    }

    public void resetHeading() {
        imu.resetYaw();
        lastHeadingDeg = startHeadingDeg;
    }

    public Action moveTo(double x, double y, double headingDeg) {
        return moveTo(x, y, headingDeg, MOVE_POWER);
    }

    public Action moveTo(double x, double y, double headingDeg, double power) {
        Move move = new Move(x, y, headingDeg, power);
        return new Action() {
            public void start() {
                move.start();
            }

            public boolean update() {
                move.execute();
                if (move.isDone()) {
                    move.end();
                    return true;
                }
                return false;
            }

            public void stop() {
                move.cancel();
            }
        };
    }

    public Action turnTo(double headingDeg) {
        Turn turn = new Turn(headingDeg);
        return new Action() {
            public void start() {
                turn.start();
            }

            public boolean update() {
                turn.execute();
                if (turn.isDone()) {
                    turn.end();
                    return true;
                }
                return false;
            }

            public void stop() {
                stopMotors();
                step = "idle";
            }
        };
    }

    public Action drive(double forward, double strafeRight, double turnRight, long timeMs) {
        ElapsedTime timer = new ElapsedTime();
        return new Action() {
            public void start() {
                timer.reset();
                readVoltage();
                step = String.format("raw drive %d ms", timeMs);
            }

            public boolean update() {
                if (timer.milliseconds() >= timeMs) {
                    stopMotors();
                    step = "idle";
                    return true;
                }
                setWheels(forward, strafeRight, turnRight);
                return false;
            }

            public void stop() {
                stopMotors();
                step = "idle";
            }
        };
    }

    public void stopMotors() {
        for (DcMotorEx m : motors) m.setPower(0);
    }

    public double getHeadingDeg() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        if (angles.getAcquisitionTime() != 0) {
            lastHeadingDeg = startHeadingDeg + angles.getYaw(AngleUnit.DEGREES);
        }
        return lastHeadingDeg;
    }

    public double getEstX() {
        return estX;
    }

    public double getEstY() {
        return estY;
    }

    public double getEstHeadingDeg() {
        return estHeadingDeg;
    }

    public String getStep() {
        return step;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive step", step);
        telemetry.addData("Estimated pose", "x %.1f  y %.1f  h %.1f", estX, estY, estHeadingDeg);
        telemetry.addData("IMU heading", "%.1f", lastHeadingDeg);
        telemetry.addData("Voltage scale", "%.3f", voltageScale);
    }

    private void readVoltage() {
        double v = voltageSensor.getVoltage();
        voltageScale = VOLTAGE_COMPENSATION && v > 6.0 ? NOMINAL_VOLTAGE / v : 1.0;
    }

    private void setWheels(double forward, double strafeRight, double turnRight) {
        double fl = forward + strafeRight + turnRight;
        double bl = forward - strafeRight + turnRight;
        double fr = forward - strafeRight - turnRight;
        double br = forward + strafeRight - turnRight;
        fl *= voltageScale;
        bl *= voltageScale;
        fr *= voltageScale;
        br *= voltageScale;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br))));
        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    private static double angleError(double targetDeg, double currentDeg) {
        double e = (targetDeg - currentDeg) % 360.0;
        if (e > 180.0) e -= 360.0;
        if (e <= -180.0) e += 360.0;
        return e;
    }

    private double holdTurn(double targetDeg, double headingDeg) {
        return Range.clip(-HOLD_KP * angleError(targetDeg, headingDeg), -HOLD_MAX, HOLD_MAX);
    }

    private class Move {
        private final double targetX, targetY, targetHeadingDeg, power;
        private final ElapsedTime timer = new ElapsedTime();
        private double ux, uy, scale, strafeRatio, durationMs;

        Move(double x, double y, double headingDeg, double power) {
            targetX = x;
            targetY = y;
            targetHeadingDeg = headingDeg;
            this.power = Range.clip(Math.abs(power), 0.0, 1.0);
        }

        void start() {
            readVoltage();
            double dx = targetX - estX;
            double dy = targetY - estY;
            double dist = Math.hypot(dx, dy);
            strafeRatio = FORWARD_IPS_PER_POWER / STRAFE_IPS_PER_POWER;
            if (dist < 0.25 || power <= 0) {
                ux = uy = scale = 0;
                durationMs = 0;
            } else {
                ux = dx / dist;
                uy = dy / dist;
                double h = Math.toRadians(targetHeadingDeg);
                double f = ux * Math.cos(h) + uy * Math.sin(h);
                double s = (ux * Math.sin(h) - uy * Math.cos(h)) * strafeRatio;
                double sum = Math.abs(f) + Math.abs(s);
                scale = Math.min(power, (1.0 - HOLD_MAX) / Math.max(sum, 1e-6));
                double cruiseMs = dist / (FORWARD_IPS_PER_POWER * scale) * 1000.0;
                if (RAMP_MS <= 0) {
                    durationMs = cruiseMs;
                } else if (cruiseMs >= RAMP_MS) {
                    durationMs = cruiseMs + RAMP_MS;
                } else {
                    durationMs = 2.0 * Math.sqrt(cruiseMs * RAMP_MS);
                }
            }
            step = String.format("moveTo (%.1f, %.1f) h %.0f  %.0f ms", targetX, targetY, targetHeadingDeg, durationMs);
            timer.reset();
        }

        void execute() {
            double t = timer.milliseconds();
            double heading = getHeadingDeg();
            if (t >= durationMs) {
                stopMotors();
                return;
            }
            double ramp = 1.0;
            if (RAMP_MS > 0) {
                ramp = Math.min(1.0, Math.min(t / RAMP_MS, (durationMs - t) / RAMP_MS));
                ramp = Math.max(0.0, ramp);
            }
            double h = Math.toRadians(heading);
            double f = (ux * Math.cos(h) + uy * Math.sin(h)) * scale * ramp;
            double s = (ux * Math.sin(h) - uy * Math.cos(h)) * strafeRatio * scale * ramp;
            setWheels(f, s, holdTurn(targetHeadingDeg, heading));
        }

        boolean isDone() {
            return timer.milliseconds() >= durationMs + SETTLE_MS;
        }

        void cancel() {
            stopMotors();
            step = "idle";
        }

        void end() {
            stopMotors();
            estX = targetX;
            estY = targetY;
            estHeadingDeg = targetHeadingDeg;
            step = "idle";
        }
    }

    private class Turn {
        private final double targetHeadingDeg;
        private final ElapsedTime timer = new ElapsedTime();
        private final ElapsedTime inToleranceTimer = new ElapsedTime();
        private boolean inTolerance = false;

        Turn(double headingDeg) {
            targetHeadingDeg = headingDeg;
        }

        void start() {
            readVoltage();
            timer.reset();
            inTolerance = false;
            step = String.format("turnTo %.0f", targetHeadingDeg);
        }

        void execute() {
            double error = angleError(targetHeadingDeg, getHeadingDeg());
            if (Math.abs(error) <= TURN_TOLERANCE_DEG) {
                if (!inTolerance) {
                    inTolerance = true;
                    inToleranceTimer.reset();
                }
                stopMotors();
                return;
            }
            inTolerance = false;
            double magnitude = Range.clip(Math.abs(error) * TURN_KP, TURN_MIN, TURN_MAX);
            setWheels(0, 0, -Math.signum(error) * magnitude);
        }

        boolean isDone() {
            return (inTolerance && inToleranceTimer.milliseconds() >= TURN_SETTLE_MS)
                    || timer.milliseconds() >= TURN_TIMEOUT_MS;
        }

        void end() {
            stopMotors();
            estHeadingDeg = targetHeadingDeg;
            step = "idle";
        }
    }
}
