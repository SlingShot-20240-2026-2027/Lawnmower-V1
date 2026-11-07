package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ButterflyDrive;
import org.firstinspires.ftc.teamcode.Subsystems.TimedDrive;

@Config
@TeleOp(name = "TimedDrive Calibration", group = "Tuning")
public class TimedDriveCalibration extends LinearOpMode {

    public static double TEST_DISTANCE_IN = 48;

    @Override
    public void runOpMode() {
        Telemetry dash = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Scheduler.reset();
        TimedDrive drive = new TimedDrive(hardwareMap, 0, 0, 90);
        ButterflyDrive butterfly = new ButterflyDrive(hardwareMap);
        butterfly.setMode(ButterflyDrive.Mode.MECANUM);

        Gamepad current = new Gamepad();
        Gamepad previous = new Gamepad();

        dash.addLine("Ready. Put the robot on the field with room in front and to the right.");
        dash.update();

        waitForStart();
        if (isStopRequested()) return;
        drive.resetHeading();

        while (opModeIsActive()) {
            previous.copy(current);
            current.copy(gamepad1);
            boolean idle = drive.getStep().equals("idle");

            if (idle && current.a && !previous.a) {
                schedule(drive.moveTo(drive.getEstX(), drive.getEstY() + TEST_DISTANCE_IN, 90));
            } else if (idle && current.b && !previous.b) {
                schedule(drive.moveTo(drive.getEstX() + TEST_DISTANCE_IN, drive.getEstY(), 90));
            } else if (idle && current.x && !previous.x) {
                schedule(drive.turnTo(180));
            } else if (idle && current.y && !previous.y) {
                schedule(drive.turnTo(90));
            }

            Scheduler.execute();

            dash.addLine("A = forward TEST_DISTANCE_IN   B = strafe right TEST_DISTANCE_IN");
            dash.addLine("X = turn to 180 (left 90)   Y = turn back to 90");
            dash.addLine("Measure how far it really went with a tape measure, then:");
            dash.addLine("  FORWARD_IPS_PER_POWER = old x measured / TEST_DISTANCE_IN");
            dash.addLine("  STRAFE_IPS_PER_POWER  = old x measured / TEST_DISTANCE_IN");
            dash.addData("FORWARD_IPS_PER_POWER", "%.2f", TimedDrive.FORWARD_IPS_PER_POWER);
            dash.addData("STRAFE_IPS_PER_POWER", "%.2f", TimedDrive.STRAFE_IPS_PER_POWER);
            dash.addData("TEST_DISTANCE_IN", "%.1f", TEST_DISTANCE_IN);
            drive.addTelemetry(dash);
            dash.update();
        }

        drive.stopMotors();
        Scheduler.reset();
    }
}
