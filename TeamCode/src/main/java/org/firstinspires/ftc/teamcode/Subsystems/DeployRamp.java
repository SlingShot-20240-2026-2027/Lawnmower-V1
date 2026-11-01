package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DeployRamp {
    public enum State {UP, DEPLOY};
    private final Servo ramp;
    private State state = State.UP;
    public DeployRamp(HardwareMap hardwareMap){ramp = hardwareMap.get(Servo.class, "deployRamp");}

    public void setState (State newState){
        state = newState;
        switch (newState){
            case UP:
                // Change servo positions once bot is done
                ramp.setPosition(0);
                break;
            case DEPLOY:
                ramp.setPosition(0.5);
                break;

        }
    }

    public Command up(){return instant(() -> setState(State.UP)).requiring(this);}

    public Command deploy(){return instant(() -> setState(State.DEPLOY)).requiring(this);}
}