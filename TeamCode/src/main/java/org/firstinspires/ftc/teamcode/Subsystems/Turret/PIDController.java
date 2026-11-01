package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {
    private double kP, kI, kD, kF;
    private double integral = 0, lastError = 0;
    private boolean first = true;
    private final ElapsedTime timer = new ElapsedTime();
    private static final double I_CLAMP = 0.4; // max integral contribution to output

    public PIDController(double p, double i, double d, double f) {
        setGains(p, i, d, f);
    }

    public void setGains(double p, double i, double d, double f) {
        kP = p; kI = i; kD = d; kF = f;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        first = true;
    }

    public double update(double target, double current, double voltage) {
        double dt = Math.max(timer.seconds(), 1e-3);
        timer.reset();

        double error = target - current;
        if (first) { lastError = error; first = false; }

        if (kI != 0) {
            integral += error * dt;
            integral = Range.clip(integral, -I_CLAMP / kI, I_CLAMP / kI);
        } else {
            integral = 0;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double out = kP * error + kI * integral + kD * derivative + kF * Math.signum(error);
        return out * (12.0 / Math.max(voltage, 8.0)); // battery compensation
    }
}