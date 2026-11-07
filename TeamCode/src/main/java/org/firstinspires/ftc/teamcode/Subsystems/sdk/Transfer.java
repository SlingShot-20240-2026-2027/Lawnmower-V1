package org.firstinspires.ftc.teamcode.Subsystems.sdk;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class Transfer {

    public static String SERVO_NAME = "transfer";
    public static boolean REVERSED = false;
    public static double FEED_POWER = 1.0;
    public static long FEED_MS = 1000;

    public enum State {FEED, REVERSE, IDLE}

    private final CRServo servo;
    private State state = State.IDLE;

    public Transfer(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        servo.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        servo.setPower(0);
    }

    public Action transfer() {
        return transfer(FEED_MS);
    }

    public Action transfer(long timeMs) {
        ElapsedTime timer = new ElapsedTime();
        return new Action() {
            public void start() {
                timer.reset();
                setState(State.FEED);
            }

            public boolean update() {
                if (timer.milliseconds() >= timeMs) {
                    setState(State.IDLE);
                    return true;
                }
                return false;
            }

            public void stop() {
                setState(State.IDLE);
            }
        };
    }

    public Action idle() {
        return Actions.instant(() -> setState(State.IDLE));
    }

    public void setState(State newState) {
        state = newState;
        double power = Range.clip(Math.abs(FEED_POWER), 0.0, 1.0);
        switch (newState) {
            case FEED:
                servo.setPower(power);
                break;
            case REVERSE:
                servo.setPower(-power);
                break;
            case IDLE:
            default:
                servo.setPower(0);
                break;
        }
    }

    public State getState() {
        return state;
    }
}
