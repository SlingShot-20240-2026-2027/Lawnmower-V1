package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.MathHelpers;

public class TurretAim {

    public static double SHOOTER_OFFSET_X = 0.566141732283465;
    public static double SHOOTER_OFFSET_Y = 0.0;

    public static double angularVelScaling = 0.07;

    public static double angleToGoal(Pose robotPose, Pose goalPose, double angularVel) {
        double cosH = Math.cos(robotPose.getHeading());
        double sinH = Math.sin(robotPose.getHeading());

        double shooterX = robotPose.getX() + SHOOTER_OFFSET_X * cosH - SHOOTER_OFFSET_Y * sinH;
        double shooterY = robotPose.getY() + SHOOTER_OFFSET_X * sinH + SHOOTER_OFFSET_Y * cosH;

        double dx = goalPose.getX() - shooterX;
        double dy = goalPose.getY() - shooterY;

        return MathHelpers.wrapAngleRadians(
                Math.atan2(dy, dx) - robotPose.getHeading() - angularVelScaling * angularVel
        );
    }
}
