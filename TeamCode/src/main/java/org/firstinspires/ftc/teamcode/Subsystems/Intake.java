package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    // Got the below tips from GEMININI for this. Ik I am such a goated coder, I always cite...
    /**
     * Pollen is "present" when the sensor reads closer than this.
     * Tune with telemetry: note the reading with no pollen and with pollen
     * in front of the sensor, then pick a value between the two.
     * The sensor has to read "clear" for this long before the next pollen can
     * count. This stops one pollen that wobbles in front of the sensor from
     * counting twice. Keep it shorter than the gap between pollen.
     */
    // Constants for intake - can be tuned (if needed)
    public static int MAX_POLLEN = 4;
    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = -1.0;
    public static double EJECT_POWER = -1.0;
    public static long EJECT_TIME_MS = 1000;
    public static double PRESENT_DISTANCE_MM = 30.0;
    public static long CLEAR_TIME_MS = 80;

    // Sensor gain
    public static float SENSOR_GAIN = 2.0f;
    public enum State {INTAKE, OUTTAKE, IDLE, EJECTING}

    private final DcMotorEx intake;
    private final RevColorSensorV3 colorSensor;

    private State state = State.IDLE;
    private State stateBeforeEject = State.INTAKE;
    private int pollenCount = 0;
    private boolean pollenAtSensor = false; // last reading
    private double lastDistanceMm = Double.NaN;

    private final ElapsedTime ejectTimer = new ElapsedTime();
    private final ElapsedTime clearTimer = new ElapsedTime();

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, "intake", "intakeColor");
    }

    public Intake(HardwareMap hardwareMap, String motorName, String sensorName) {
        intake = hardwareMap.get(DcMotorEx.class, motorName);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colorSensor = hardwareMap.get(RevColorSensorV3.class, sensorName);
        colorSensor.setGain(SENSOR_GAIN);
        colorSensor.enableLed(true);
    }

    public void setState(State newState) {
        if (state == State.EJECTING && newState == State.INTAKE) {
            stateBeforeEject = State.INTAKE;
            return;
        }
        state = newState;
        applyPower();
    }

    public State getState() {
        return state;
    }

    private void applyPower() {
        switch (state) {
            case INTAKE:
                intake.setPower(INTAKE_POWER);
                break;
            case OUTTAKE:
                intake.setPower(OUTTAKE_POWER);
                break;
            case EJECTING:
                intake.setPower(EJECT_POWER);
                break;
            case IDLE:
            default:
                intake.setPower(0);
                break;
        }
    }

    // Sensor -> auto eject
    public void update() {
        lastDistanceMm = colorSensor.getDistance(DistanceUnit.MM);
        boolean rawPresent = lastDistanceMm < PRESENT_DISTANCE_MM;

        // Needs to clear timer for reuse
        boolean newPollenArrived = false;
        if (rawPresent) {
            if (!pollenAtSensor) {
                newPollenArrived = true;
            }
            pollenAtSensor = true;
            clearTimer.reset();
        } else if (pollenAtSensor && clearTimer.milliseconds() >= CLEAR_TIME_MS) {
            pollenAtSensor = false;
        }

        switch (state) {
            case INTAKE:
                if (newPollenArrived) {
                    if (pollenCount < MAX_POLLEN) {
                        pollenCount++;           // room for it, keep it
                    }
                    else {
                        startEject();            // over the limit, spit it out
                    }
                }
                break;

            case EJECTING:
                // Only after timer is over and sensor is clear
                if (ejectTimer.milliseconds() >= EJECT_TIME_MS && !pollenAtSensor) {
                    state = stateBeforeEject;
                    applyPower();
                }
                break;

            case OUTTAKE:
            case IDLE:
            default:
                break;
        }
    }

    private void startEject() {
        stateBeforeEject = State.INTAKE;
        state = State.EJECTING;
        ejectTimer.reset();
        applyPower();
    }

    // Pollen counter

    public int getPollenCount() {
        return pollenCount;
    }

    public boolean isFull() {
        return pollenCount >= MAX_POLLEN;
    }

    // When 1 pollen leaves robot
    public void pollenScored() {
        pollenCount = Math.max(0, pollenCount - 1);
    }

    // Sets count
    public void setPollenCount(int count) {
        pollenCount = Math.max(0, Math.min(MAX_POLLEN, count));
    }

    public void resetPollenCount() {
        pollenCount = 0;
    }

    public void addTelemetry(Telemetry telemetry) {
        NormalizedRGBA c = colorSensor.getNormalizedColors();
        telemetry.addData("Intake State", state);
        telemetry.addData("Pollen", "%d / %d%s", pollenCount, MAX_POLLEN, isFull() ? "  FULL" : "");
        telemetry.addData("Sensor Dist (mm)", "%.1f", lastDistanceMm);
        telemetry.addData("Pollen At Sensor", pollenAtSensor);
        telemetry.addData("Sensor RGB", "%.3f %.3f %.3f", c.red, c.green, c.blue);
    }


    public Command in() {
        return instant(() -> setState(State.INTAKE)).requiring(this);
    }

    public Command out() {
        return instant(() -> setState(State.OUTTAKE)).requiring(this);
    }

    public Command idle() {
        return instant(() -> setState(State.IDLE)).requiring(this);
    }

    public Command scored() {
        return instant(this::pollenScored);
    }
}
