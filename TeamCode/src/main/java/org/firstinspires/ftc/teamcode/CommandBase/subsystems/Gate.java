package org.firstinspires.ftc.teamcode.CommandBase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.GATE_OPEN;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.GATE_CLOSED;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

public class Gate implements Subsystem{

    public enum State { OPEN, CLOSE}

    private final Servo gate;

    private State state = State.CLOSE;
    public Gate(RobotHardware robotHardware) {this.gate = robotHardware.gate;}

    @Override
    public void init() {}

    @Override
    public void update() {
        switch (state) {
            case OPEN:      gate.setPosition(GATE_OPEN);  break;
            case CLOSE:     gate.setPosition(GATE_CLOSED); break;
        }
    }
    public void open()   { state = Gate.State.CLOSE; }
    public void close()  { state = Gate.State.OPEN;}
}
