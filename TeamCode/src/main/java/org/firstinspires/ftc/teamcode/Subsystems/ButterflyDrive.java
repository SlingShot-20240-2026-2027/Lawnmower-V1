package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.instant;

public class ButterflyDrive {
    public static String LEFT_SERVO_NAME  = "leftButterflyServo";
    public static String RIGHT_SERVO_NAME = "rightButterflyServo";
    public static double LEFT_RETRACTED  = 0.20;
    public static double LEFT_DEPLOYED   = 0.75;
    public static double RIGHT_RETRACTED = 0.80;
    public static double RIGHT_DEPLOYED  = 0.25;

    public static long TRANSITION_TIME_MS = 300;

    // Modes
    public enum Mode {
        MECANUM,
        TRACTION
    }

    private final Servo leftServo;
    private final Servo rightServo;
    private final ElapsedTime transitionTimer = new ElapsedTime();

    private Mode mode = Mode.MECANUM;

    public ButterflyDrive(HardwareMap hardwareMap) {
        leftServo  = hardwareMap.get(Servo.class, LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(Servo.class, RIGHT_SERVO_NAME);

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.FORWARD);
    }

    // Set Hardware Positions According to Mode
    public void setMode(Mode newMode) {
        if (newMode == Mode.TRACTION) {
            leftServo.setPosition(LEFT_DEPLOYED);
            rightServo.setPosition(RIGHT_DEPLOYED);
        } else {
            leftServo.setPosition(LEFT_RETRACTED);
            rightServo.setPosition(RIGHT_RETRACTED);
        }
        if (newMode != mode) {
            transitionTimer.reset();
        }
        mode = newMode;
    }

    public Command deployCommand() {
        return instant(() -> setMode(Mode.TRACTION));
    }
    public Command retractCommand() {
        return instant(() -> setMode(Mode.MECANUM));
    }
    public Command toggleCommand() {
        return instant(() -> setMode(mode == Mode.MECANUM ? Mode.TRACTION:Mode.MECANUM));
    }

// Telemetry Tracking
    public Mode getMode() {
        return mode;
    }

    public boolean isTraction() {
        return mode == Mode.TRACTION;
    }

    public boolean isSettled() {
        return transitionTimer.milliseconds() >= TRANSITION_TIME_MS;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Butterfly Mode", mode);
        telemetry.addData("Butterfly Settled", isSettled());
        telemetry.addData("Butterfly Servos", "%.2f / %.2f",
                leftServo.getPosition(), rightServo.getPosition());
    }
}