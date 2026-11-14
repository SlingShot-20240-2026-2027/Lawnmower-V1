package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    // Intake and Out take
    public static final double IN_POWER = 1;
    public static final double OUT_POWER = -1;
    //Gate
    public static final double GATE_OPEN = 0.4;
    public static final double GATE_CLOSED = 0.66;
    //Butterfly
    public static final double DEPLOY_BUTTERFLY1 = 0.24;
    public static final double IDLE_BUTTERFLY1 = 0.45;
    public static final double DEPLOY_BUTTERFLY2 = 0.125;
    public static final double IDLE_BUTTERFLY2 = 0;
    // Shooter
    public static final double SHOOTER_VELOCITY = 855;
    public static final double P = 83.5;
    public static final double I = 0;
    public static final double D = 0;
    public static final double F = 12.5;
    // Hood
    public static final double HOOD_POSITION = 0.33;
}

