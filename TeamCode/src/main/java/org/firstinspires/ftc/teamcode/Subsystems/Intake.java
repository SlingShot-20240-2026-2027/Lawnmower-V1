package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public enum State {IN, OUT, IDLE};

    public static double IN_POWER = 1.0;
    public static double OUT_POWER = -1.0;

    private final DcMotorEx motor;
    private State state = State.IDLE;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // motor.setDirection(DcMotorSimple.Direction.REVERSE); // flip if it spins the wrong way
    }

    public void setState(State newState) {
        state = newState;
        switch (newState) {
            case IN:
                motor.setPower(IN_POWER);
                break;
            case OUT:
                motor.setPower(OUT_POWER);
                break;
            case IDLE:
                motor.setPower(0);
                break;
        }
    }

    public State getState() { return state; }

    public Command in()   { return instant(() -> setState(State.IN)).requiring(this); }

    public Command out()  { return instant(() -> setState(State.OUT)).requiring(this); }

    public Command idle() { return instant(() -> setState(State.IDLE)).requiring(this); }
}