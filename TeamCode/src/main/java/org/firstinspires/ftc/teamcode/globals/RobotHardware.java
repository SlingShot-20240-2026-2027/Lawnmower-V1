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


    private RobotHardware(OpMode opMode) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        intake = hardwareMap.get(DcMotor.class,"intake");
        transfer = hardwareMap.get(DcMotor.class,"transfer");

        gate = hardwareMap.get(Servo.class,"gate");

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