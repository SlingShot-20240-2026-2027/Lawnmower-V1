package org.firstinspires.ftc.teamcode.CommandBase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.D;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.F;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.I;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.P;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.SHOOTER_VELOCITY;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.globals.RobotHardware;

public class Shooter implements Subsystem {

    public enum State { ON, IDLE }

    private static final double VELOCITY_TOLERANCE = 50; // ticks/sec, tune as needed

    private final DcMotorEx shooter1, shooter2;

    private State state = State.IDLE;



    public Shooter(RobotHardware robotHardware) {
        this.shooter1 = robotHardware.f1;
        this.shooter2 = robotHardware.f2;
    }

    @Override
    public void init() {
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);

        for (DcMotorEx motor : new DcMotorEx[]{shooter2}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }
    }

    @Override
    public void update() {
        switch (state) {
            case ON:
                shooter1.setVelocity(SHOOTER_VELOCITY);
                shooter2.setVelocity(SHOOTER_VELOCITY);
                break;
            case IDLE:
                shooter1.setPower(0);
                shooter2.setPower(0);
                break;
        }
    }

    public void on()   { state = State.ON; }
    public void idle() { state = State.IDLE; }

    public State getState() { return state; }

    public boolean atTargetVelocity() {
        return state == State.ON
                && Math.abs(shooter1.getVelocity() - SHOOTER_VELOCITY) < VELOCITY_TOLERANCE
                && Math.abs(shooter2.getVelocity() - SHOOTER_VELOCITY) < VELOCITY_TOLERANCE;
    }
}