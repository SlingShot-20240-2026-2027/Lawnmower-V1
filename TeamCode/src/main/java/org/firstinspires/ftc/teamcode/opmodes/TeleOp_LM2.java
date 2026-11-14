package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.HOOD_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBase.subsystems.Butterfly;
import org.firstinspires.ftc.teamcode.CommandBase.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.CommandBase.subsystems.Gate;
import org.firstinspires.ftc.teamcode.CommandBase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.CommandBase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.CommandBase.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.CommandBase.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

@TeleOp(name = "TeleOp LM2")
public class TeleOp_LM2 extends OpMode {

    private RobotHardware robot;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Gate gate;
    private Intake intake;
    private Butterfly butterfly;
    private Subsystem[] subsystems;
    private DcMotor transfer;

    private Servo hood;

    private boolean lastRB = false;
    private boolean butterflyToggled = false;

    private Servo deployable;

    private boolean lastLB = false;
    private boolean atZero = false; // first press goes to 0

    private boolean lastA = false;
    private boolean running = false;

    private Servo led;

    @Override
    public void init() {
        robot = RobotHardware.getInstance(this);

        drivetrain = new Drivetrain(robot);
        shooter = new Shooter(robot);
        gate = new Gate(robot);
        intake = new Intake(robot);
        butterfly = new Butterfly(robot);

        subsystems = new Subsystem[]{drivetrain, shooter, gate, intake,butterfly};
        for (Subsystem s : subsystems) s.init();


        hood = hardwareMap.get(Servo.class,"hood");
        transfer = hardwareMap.get(DcMotor.class,"transfer");
        deployable = hardwareMap.get(Servo.class,"deployable");
        deployable.setPosition(0);

        led = hardwareMap.get(Servo.class,"led");

    }

    @Override
    public void loop() {
        boolean shootHeld = gamepad1.right_trigger > 0.5;
        hood.setPosition(HOOD_POSITION);
        boolean lb = gamepad1.left_bumper;
        shooter.on(); // shooter always runs

        // Gate: open only while right trigger is held to shoot
        if (shootHeld) {
            gate.open();
            led.setPosition(0.722);
        } else {
            gate.close();
            led.setPosition(0);
        }
        // Butterfly: rb toggles manually; right trigger forces it down while held
        boolean rb = gamepad1.right_bumper;
        if (rb && !lastRB) butterflyToggled = !butterflyToggled;
        lastRB = rb;

        if (shootHeld || butterflyToggled) butterfly.deploy();
        else butterfly.idle();

        if (lb && !lastLB) {
            atZero = !atZero;
            deployable.setPosition(atZero ? 1 : 0);
        }
        lastLB = lb;

        // Drive: left stick translates, right stick turns
        drivetrain.drive(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        // reverse intake when dpad down, else always run
        if (gamepad1.dpad_down) {
            intake.out();
            transfer.setPower(-1);
        }

        //intake logic
        boolean a = gamepad1.a;
        if (a && !lastA) running = !running;
        lastA = a;

        if (running) {
            intake.in();
            transfer.setPower(1);
        } else {
            intake.idle();
            transfer.setPower(0);
        }

        intake.update();


        for (Subsystem s : subsystems) s.update();

        telemetry.addData("F1 vel", robot.f1.getVelocity());
        telemetry.addData("F2 vel", robot.f2.getVelocity());
        telemetry.update();
    }
}