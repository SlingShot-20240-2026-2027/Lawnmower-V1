package org.firstinspires.ftc.teamcode.CommandBase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.IN_POWER;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.OUT_POWER;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

public class Intake implements Subsystem {

    public enum State { IN, OUT, IDLE }

    private final DcMotor intake;
    private State state = State.IDLE;

    public Intake(RobotHardware robotHardware) {this.intake = robotHardware.intake;}
    @Override
    public void init() {}
    @Override
    public void update() {
        switch (state) {
            case IN:      intake.setPower(IN_POWER);  break;
            case OUT:     intake.setPower(OUT_POWER); break;
            case IDLE:    intake.setPower(0);         break;
        }
    }

    public void in()   { state = State.IN; }
    public void out()  { state = State.OUT; }
    public void idle() { state = State.IDLE; }
}