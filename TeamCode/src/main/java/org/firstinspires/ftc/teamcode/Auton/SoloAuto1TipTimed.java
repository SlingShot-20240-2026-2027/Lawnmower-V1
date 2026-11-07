package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ButterflyDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooternf;
import org.firstinspires.ftc.teamcode.Subsystems.TimedDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

@Autonomous(name = "SoloAuto1Tip Timed", group = "Autonomous")
public class SoloAuto1TipTimed extends LinearOpMode {

    private static final double START_X = 55.92;
    private static final double START_Y = 9.218;
    private static final double START_HEADING = 90;

    private TimedDrive drive;
    private ButterflyDrive butterfly;
    private Shooternf shooter;
    private Transfer transfer;
    private Intake intake;
    private Gate gate;

    public Command autoRoutine() {
        return sequential(
                parallel(
                        shooter.on(),
                        intake.in(),
                        transfer.transfer(),
                        gate.cl()
                ),
                waitMs(2000),
                gate.op(),
                waitMs(1000),
                gate.cl(),
                drive.moveTo(40.81, 18.14, 90),
                drive.turnTo(270),
                drive.moveTo(8.75, 29.39, 270),
                drive.moveTo(8.75, 8.78, 270),
                drive.moveTo(25.93, 61.67, 270),
                drive.moveTo(13.69, 95.71, 270),
                parallel(
                        transfer.idle(),
                        shooter.idle(),
                        intake.idle()
                )
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        drive = new TimedDrive(hardwareMap, START_X, START_Y, START_HEADING);
        butterfly = new ButterflyDrive(hardwareMap);
        shooter = new Shooternf(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);

        butterfly.setMode(ButterflyDrive.Mode.MECANUM);
        gate.setState(Gate.State.CLOSED);

        telemetry.addLine("Ready. Robot at the start spot, facing 90 deg. Keep it still.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        drive.resetHeading();
        schedule(shooter.periodic());
        schedule(autoRoutine());

        while (opModeIsActive()) {
            intake.update();
            Scheduler.execute();

            drive.addTelemetry(telemetry);
            telemetry.addData("Shooter", "%.0f / %.0f rpm  ready %s",
                    shooter.getRpm(), shooter.getTargetRpm(), shooter.isReady());
            telemetry.addData("Pollen", intake.getPollenCount());
            telemetry.update();
        }

        drive.stopMotors();
        shooter.stop();
        transfer.setState(Transfer.State.IDLE);
        intake.setState(Intake.State.IDLE);
    }
}
