package org.firstinspires.ftc.teamcode.CommandBase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.globals.RobotHardware;

public class Drivetrain implements Subsystem {

    private final DcMotorEx fl, fr, bl, br;

    private double x, y, rx;

    public Drivetrain(RobotHardware robotHardware) {
        this.fl = robotHardware.fl;
        this.fr = robotHardware.fr;
        this.bl = robotHardware.bl;
        this.br = robotHardware.br;
    }

    @Override
    public void init() {
        for (DcMotorEx motor : new DcMotorEx[]{fl, fr, bl, br}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /** x = strafe (right +), y = forward (+), rx = turn (clockwise +). */
    public void drive(double x, double y, double rx) {
        this.x = x;
        this.y = y;
        this.rx = rx;
    }

    @Override
    public void update() {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        fl.setPower((y + x + rx) / denominator);
        bl.setPower((y - x + rx) / denominator);
        fr.setPower((y - x - rx) / denominator);
        br.setPower((y + x - rx) / denominator);
    }
}