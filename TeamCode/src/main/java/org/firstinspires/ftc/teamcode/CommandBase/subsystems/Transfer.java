package org.firstinspires.ftc.teamcode.CommandBase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.IN_POWER;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.OUT_POWER;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

public class Transfer implements Subsystem {

    public enum State { IN, OUT, IDLE }

    private final DcMotor transfer;
    private State state = State.IDLE;

    public Transfer(RobotHardware robotHardware) {
        this.transfer = robotHardware.intake;
    }
    @Override
    public void init() {}
    @Override
    public void update() {
        switch (state) {
            case IN:      transfer.setPower(IN_POWER);  break;
            case OUT:     transfer.setPower(OUT_POWER); break;
            case IDLE:    transfer.setPower(0);         break;
        }
    }

    public void in()   { state = State.IN; }
    public void out()  { state = State.OUT; }
    public void idle() { state = State.IDLE; }
}