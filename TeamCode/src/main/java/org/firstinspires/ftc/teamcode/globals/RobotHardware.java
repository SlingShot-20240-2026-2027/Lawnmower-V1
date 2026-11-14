package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class RobotHardware {

    private static RobotHardware instance;

    private final OpMode opMode;

    public DcMotor intake , transfer;

    public Servo gate;

    public Servo b1, b2;
    public DcMotorEx f1, f2;
    public DcMotorEx fl, fr, bl, br;


    private RobotHardware(OpMode opMode) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        intake = hardwareMap.get(DcMotor.class,"intake");
        transfer = hardwareMap.get(DcMotor.class,"transfer");
        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.REVERSE);

        gate = hardwareMap.get(Servo.class,"gate");

        b1 = hardwareMap.get(Servo.class,"butterfly1");
        b2 = hardwareMap.get(Servo.class,"butterfly2");

        f1 = hardwareMap.get(DcMotorEx.class,"flywheel1");
        f2 = hardwareMap.get(DcMotorEx.class,"flywheel2");
        f1.setDirection(DcMotorEx.Direction.REVERSE);
        f2.setDirection(DcMotorEx.Direction.REVERSE);

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    //copied from off season repo
    public static RobotHardware getInstance(OpMode opMode) {
        if (opMode == null) {
            return null;
        }
        // Rebuild whenever a new OpMode asks, so devices are re-fetched and motor
        // directions are re-applied after the SDK resets them between runs
        if (instance == null || instance.opMode != opMode) {
            instance = new RobotHardware(opMode);
        }
        return instance;
    }
}