package org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class TurretConstants {
    // ---------- Encoder (analog absolute) ----------
    public static double ENCODER_MAX_VOLTAGE = 3.3;
    public static double ENCODER_ZERO_DEG = 0;          // raw encoder deg when turret faces forward
    public static double ENCODER_WRAP_OFFSET_DEG = 0;   // moves the 0/360 seam outside the turret's travel range
    public static double ENCODER_TO_TURRET_RATIO = 1.0; // encoder revs per turret rev

    // ---------- Travel limits (deg, turret frame, CCW+) ----------
    public static double SOFT_MIN_DEG = -200;
    public static double SOFT_MAX_DEG = 200;
    public static double FORWARD_OFFSET_DEG = 0;
    public static double HOME_DEG = 0;

    // ---------- PID: close (fine) and far (coarse) ----------
    public static double P_CLOSE = 0.015, I_CLOSE = 0.002, D_CLOSE = 0.0008, F_CLOSE = 0.04;
    public static double P_FAR   = 0.010, I_FAR   = 0.0,   D_FAR   = 0.0005, F_FAR   = 0.05;
    public static double PID_SWITCH_DEG = 8;
    public static double PID_SWITCH_HYSTERESIS_DEG = 2;
    public static boolean PID_OUTPUT_INVERT = false;

    public static double MAX_POWER = 1.0;
    public static double SERVO_POWER_THRESHOLD = 0.005; // skip redundant hub writes

    // ---------- Slew limiting ----------
    public static boolean SLEW_ENABLED = true;
    public static double SLEW_DEG_PER_SEC = 720;

    // ---------- Angular velocity lead ----------
    public static double ANG_VEL_LEAD_SEC = 0.05;

    // ---------- Limelight localization (start of match) ----------
    public static int    TAG_ID = 20;
    public static double TAG_BEARING_DEG = 0;       // tag direction relative to robot at start (CCW+)
    public static int    RELOC_SAMPLES = 5;         // distinct Limelight frames to average
    public static double LOCALIZE_TIMEOUT_SEC = 1.0; // give up and trust the encoder after this long
}