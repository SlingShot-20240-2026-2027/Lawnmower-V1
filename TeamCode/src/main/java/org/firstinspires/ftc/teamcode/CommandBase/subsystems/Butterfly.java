package org.firstinspires.ftc.teamcode.CommandBase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.DEPLOY_BUTTERFLY1;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.DEPLOY_BUTTERFLY2;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.GATE_OPEN;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.GATE_CLOSED;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.IDLE_BUTTERFLY1;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.IDLE_BUTTERFLY2;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

public class Butterfly implements Subsystem{

    public enum State { DEPLOY, IDLE}

    private final Servo b1 , b2;

    private State state = State.IDLE;
    public Butterfly(RobotHardware robotHardware) {
        this.b1 = robotHardware.b1;
        this.b2 = robotHardware.b2;
    }

    @Override
    public void init() {}

    @Override
    public void update() {
        switch (state) {
            case DEPLOY:
                b1.setPosition(DEPLOY_BUTTERFLY1);
                b2.setPosition(DEPLOY_BUTTERFLY2);
                break;
            case IDLE:
                b1.setPosition(IDLE_BUTTERFLY1);
                b2.setPosition(IDLE_BUTTERFLY2);
                break;
        }
    }
    public void deploy()   { state = Butterfly.State.DEPLOY; }
    public void idle()  { state = Butterfly.State.IDLE;}
}

