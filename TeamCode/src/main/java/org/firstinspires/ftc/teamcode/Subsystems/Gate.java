package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {

    public enum State {CLOSED, OPEN};
    private final Servo gate;
    private State state = State.CLOSED;
    public Gate(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, "gate");
    }

    public void setState (State newState){
        state = newState;
        switch (newState){
            case CLOSED:
                // Change servo positions once bot is done
                gate.setPosition(0);
                break;
            case OPEN:
                gate.setPosition(0.5);
                break;
        }
    }

    public Command cl(){return instant(() -> setState(State.CLOSED)).requiring(this);}

    public Command op(){return instant(() -> setState(State.OPEN)).requiring(this);}
}