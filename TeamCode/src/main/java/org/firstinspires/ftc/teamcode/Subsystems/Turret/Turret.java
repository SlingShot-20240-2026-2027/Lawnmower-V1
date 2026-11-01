package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Turret.PIDController;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretConstants.*;

public class Turret {

    private final CRServo servo1, servo2, servo3;
    private final AnalogInput encoder;
    private final Limelight3A limelight;
    private final PIDController pidClose, pidFar;

    private boolean tracking = false;
    private boolean useClosePid = false;
    private boolean localized = false;

    private double targetAngle = HOME_DEG;      // includes userOffset
    private double commandedAngle = HOME_DEG;   // slewed command
    private double userOffsetDeg = 0;

    private double absAngleDeg = 0;             // continuous turret angle (unwrapped)
    private double lastShiftedDeg;
    private double lastPower = Double.NaN;

    private final ElapsedTime slewTimer = new ElapsedTime();

    // start-of-match localization state
    private boolean locAttempting = false;
    private final ElapsedTime locTimer = new ElapsedTime();
    private double locSum = 0;
    private int locCount = 0;
    private double lastLocTimestamp = -1;

    // debug
    public double dbg_raw, dbg_abs, dbg_target, dbg_error, dbg_pidOut, dbg_power, dbg_rawTarget;

    public Turret(HardwareMap hw) {
        servo1 = hw.get(CRServo.class, "servo1");
        servo2 = hw.get(CRServo.class, "servo2");
        servo3 = hw.get(CRServo.class, "servo3");
        servo2.setDirection(DcMotorSimple.Direction.REVERSE); // flip whichever are mirrored

        encoder = hw.get(AnalogInput.class, "turretEncoder");

        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);   // AprilTag pipeline
        limelight.start();             // warmed up before Play

        pidClose = new PIDController(P_CLOSE, I_CLOSE, D_CLOSE, F_CLOSE);
        pidFar   = new PIDController(P_FAR, I_FAR, D_FAR, F_FAR);

        seedEncoder();
        targetAngle = commandedAngle = absAngleDeg;
        slewTimer.reset();
    }

    // ---------------- Public API ----------------
    public void startTrack() { tracking = true; }
    public void stopTrack()  { tracking = false; }

    public void setAngle(double deg)        { targetAngle = deg + userOffsetDeg; }
    public void resetTurret()               { targetAngle = HOME_DEG; }
    public void addOffsetDegrees(double d)  { userOffsetDeg += d; }
    public void resetOffsetDegrees()        { userOffsetDeg = 0; }
    public double getOffsetDegrees()        { return userOffsetDeg; }
    public double getCurrentAngle()         { return absAngleDeg; }
    public double getTargetAngle()          { return targetAngle; }
    public boolean isLocalized()            { return localized; }
    public boolean atTarget(double tolDeg)  { return Math.abs(targetAngle - absAngleDeg) < tolDeg; }

    public void setManualPower(double p) {
        pidClose.reset();
        pidFar.reset();
        setServoPower(Range.clip(p, -1, 1));
    }

    public void stop() { setManualPower(0); }

    /** Force the turret's known angle (e.g. restoring from auto to teleop). */
    public void setAbsoluteAngle(double deg) {
        absAngleDeg = deg;
        targetAngle = commandedAngle = deg;
    }

    // ---------------- Start-of-match Limelight localization ----------------
    /** Call once, right when the match starts. Holds the turret still while it looks for the tag. */
    public void beginMatchLocalization() {
        stop();
        locAttempting = true;
        localized = false;
        locSum = 0;
        locCount = 0;
        lastLocTimestamp = -1;
        locTimer.reset();
    }

    /** Call every loop after beginMatchLocalization(). Returns true when finished (success or timeout). */
    public boolean updateLocalization() {
        if (!locAttempting) return true;

        readEncoderUnwrapped(); // keep the encoder tracker fresh while waiting

        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid() && r.getTimestamp() != lastLocTimestamp) {
            for (LLResultTypes.FiducialResult t : r.getFiducialResults()) {
                if (t.getFiducialId() == TAG_ID) {
                    locSum += t.getTargetXDegrees();
                    locCount++;
                    lastLocTimestamp = r.getTimestamp(); // count each frame once
                    break;
                }
            }
        }

        if (locCount >= RELOC_SAMPLES) {
            // Camera rides on the turret: tagBearing = turretAngle - tx  ->  turretAngle = bearing + tx
            double trueAngle = TAG_BEARING_DEG + locSum / locCount;
            setAbsoluteAngle(trueAngle);   // encoder now reports the true turret angle
            pidClose.reset();
            pidFar.reset();
            localized = true;
            locAttempting = false;
            return true;
        }

        if (locTimer.seconds() > LOCALIZE_TIMEOUT_SEC) {
            // No tag seen: fall back to the encoder's own reading
            targetAngle = commandedAngle = absAngleDeg;
            locAttempting = false;
            return true;
        }
        return false;
    }

    // ---------------- Encoder ----------------
    private static double wrap360(double d) { return ((d % 360) + 360) % 360; }
    private static double wrap180(double d) { return wrap360(d + 180) - 180; }

    private double readRawDeg() {
        return wrap360(encoder.getVoltage() / ENCODER_MAX_VOLTAGE * 360.0);
    }

    private double shifted(double rawDeg) { return wrap360(rawDeg + ENCODER_WRAP_OFFSET_DEG); }

    private void seedEncoder() {
        double raw = readRawDeg();
        absAngleDeg = wrap180(raw - ENCODER_ZERO_DEG) / ENCODER_TO_TURRET_RATIO;
        lastShiftedDeg = shifted(raw);
        dbg_raw = raw;
    }

    private double readEncoderUnwrapped() {
        double raw = readRawDeg();
        double s = shifted(raw);
        double delta = wrap180(s - lastShiftedDeg);
        absAngleDeg += delta / ENCODER_TO_TURRET_RATIO;
        lastShiftedDeg = s;
        dbg_raw = raw;
        dbg_abs = absAngleDeg;
        return absAngleDeg;
    }

    /** Read the encoder without driving (use while pushing the turret by hand). */
    public double updateEncoderOnly() {
        double a = readEncoderUnwrapped();
        targetAngle = commandedAngle = a;
        return a;
    }

    // ---------------- Control ----------------
    private double remapIntoSafeRange(double desired, double reference) {
        int kMin = (int) Math.ceil((SOFT_MIN_DEG - desired) / 360.0);
        int kMax = (int) Math.floor((SOFT_MAX_DEG - desired) / 360.0);
        if (kMin > kMax) return Range.clip(desired, SOFT_MIN_DEG, SOFT_MAX_DEG);

        double best = desired + 360.0 * kMin;
        for (int k = kMin + 1; k <= kMax; k++) {
            double c = desired + 360.0 * k;
            if (Math.abs(c - reference) < Math.abs(best - reference)) best = c;
        }
        return Range.clip(best, SOFT_MIN_DEG, SOFT_MAX_DEG);
    }

    public void update(double dx, double dy, double headingRad) {
        update(dx, dy, headingRad, 0, 12.0);
    }

    /**
     * @param dx,dy        vector from robot to goal (field frame)
     * @param headingRad   robot heading, radians
     * @param angVelRadSec robot angular velocity, for lead compensation
     * @param voltage      battery voltage
     */
    public void update(double dx, double dy, double headingRad, double angVelRadSec, double voltage) {
        if (tracking) {
            double leadDeg = Math.toDegrees(angVelRadSec) * ANG_VEL_LEAD_SEC;
            double robotHeading = Math.toDegrees(headingRad) + leadDeg;
            double headingToGoal = Math.toDegrees(Math.atan2(dy, dx));
            double rawTarget = headingToGoal - robotHeading + FORWARD_OFFSET_DEG;
            dbg_rawTarget = rawTarget;

            // shortest path from current commanded angle
            double cur = commandedAngle - userOffsetDeg;
            double diff = rawTarget - cur;
            diff -= 360 * Math.round(diff / 360.0);
            setAngle(cur + diff);
        }
        runControl(voltage);
    }

    /** Hold/move to targetAngle without tracking math. */
    public void update(double voltage) { runControl(voltage); }

    private void runControl(double voltage) {
        targetAngle = remapIntoSafeRange(targetAngle, commandedAngle);

        double cmd = targetAngle;
        double dt = slewTimer.seconds();
        slewTimer.reset();
        if (SLEW_ENABLED) {
            double maxDelta = SLEW_DEG_PER_SEC * dt;
            cmd = commandedAngle + Range.clip(targetAngle - commandedAngle, -maxDelta, maxDelta);
        }
        commandedAngle = cmd;

        double angle = readEncoderUnwrapped();
        double error = cmd - angle;

        // close/far PID with hysteresis
        double enter = PID_SWITCH_DEG - PID_SWITCH_HYSTERESIS_DEG;
        double exit  = PID_SWITCH_DEG + PID_SWITCH_HYSTERESIS_DEG;
        if (useClosePid) { if (Math.abs(error) > exit) useClosePid = false; }
        else             { if (Math.abs(error) < enter) useClosePid = true; }

        double out = useClosePid
                ? pidClose.update(cmd, angle, voltage)
                : pidFar.update(cmd, angle, voltage);

        dbg_target = cmd;
        dbg_error = error;
        dbg_pidOut = out;

        if (PID_OUTPUT_INVERT) out = -out;
        out = Range.clip(out, -MAX_POWER, MAX_POWER);

        // hard stop at soft limits
        if ((angle >= SOFT_MAX_DEG && out > 0) || (angle <= SOFT_MIN_DEG && out < 0)) out = 0;

        setServoPower(out);
    }

    private void setServoPower(double p) {
        if (Double.isNaN(lastPower) || Math.abs(p - lastPower) > SERVO_POWER_THRESHOLD) {
            servo1.setPower(p);
            servo2.setPower(p);
            servo3.setPower(p);
            lastPower = p;
        }
        dbg_power = p;
    }
}