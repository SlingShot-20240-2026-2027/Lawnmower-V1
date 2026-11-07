package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.teamcode.Subsystems.sdk.Actions.instant;
import static org.firstinspires.ftc.teamcode.Subsystems.sdk.Actions.sequential;
import static org.firstinspires.ftc.teamcode.Subsystems.sdk.Actions.waitMs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ButterflyDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.sdk.Action;
import org.firstinspires.ftc.teamcode.Subsystems.sdk.Actions;
import org.firstinspires.ftc.teamcode.Subsystems.sdk.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.sdk.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.sdk.Transfer;

@Autonomous(name = "SoloAuto SDK", group = "Autonomous")
public class SoloAutoSDK extends LinearOpMode {

    private static final double START_X = 55.92;
    private static final double START_Y = 9.218;
    private static final double START_HEADING = 90;

    private Drive drive;
    private ButterflyDrive butterfly;
    private Shooter shooter;
    private Transfer transfer;
    private Intake intake;

    public Action autoRoutine() {
        return sequential(
                intakeOn(),
                shooter.on(),
                waitMs(3000),
                transfer.transfer(),
                shooter.idle(),
                drive.moveTo(40.81, 18.14, 90),
                drive.turnTo(270),
                drive.moveTo(8.75, 29.39, 270),
                drive.moveTo(8.75, 8.78, 270),
                drive.moveTo(25.93, 61.67, 270),
                drive.moveTo(13.69, 95.71, 270),
                intakeOff()
        );
    }

    private Action intakeOn() {
        return instant(() -> intake.setState(Intake.State.INTAKE));
    }

    private Action intakeOff() {
        return instant(() -> intake.setState(Intake.State.IDLE));
    }

    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap, START_X, START_Y, START_HEADING);
        butterfly = new ButterflyDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);

        butterfly.setMode(ButterflyDrive.Mode.MECANUM);

        telemetry.addLine("Ready. Robot at the start spot, facing 90 deg. Keep it still.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        drive.resetHeading();
        Actions.run(this, autoRoutine(), this::everyLoop);

        while (opModeIsActive()) {
            everyLoop();
        }

        drive.stopMotors();
        shooter.stop();
        transfer.setState(Transfer.State.IDLE);
        intake.setState(Intake.State.IDLE);
    }

    private void everyLoop() {
        shooter.update();
        intake.update();
        drive.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        telemetry.addData("Transfer", transfer.getState());
        telemetry.addData("Pollen", intake.getPollenCount());
        telemetry.update();
    }
}
