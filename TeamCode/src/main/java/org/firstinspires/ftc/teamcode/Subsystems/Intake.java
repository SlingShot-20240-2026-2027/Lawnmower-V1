package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public enum State {INTAKE, OUTTAKE, IDLE}
    private final DcMotorEx intake;
    private State state = State.IDLE;

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, "intake");
    }
    public Intake(HardwareMap hardwareMap, String name) {
        intake = hardwareMap.get(DcMotorEx.class, name);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setState(State newState) {
        state = newState;
        switch(newState) {
            case INTAKE:
                intake.setPower(1.0);
                break;
            case OUTTAKE:
                intake.setPower(-1.0);
                break;
            case IDLE:
            default:
                intake.setPower(0);
                break;
        }
    }

    public Intake.State getState() {
        return state;
    }
    public Command in() {
        return instant(() -> setState(State.INTAKE)).requiring(this);
    }
    public Command out() {
        return instant(() -> setState(State.OUTTAKE)).requiring(this);
    }
    public Command idle() {
        return instant(() -> setState(State.IDLE)).requiring(this);

    }
}
