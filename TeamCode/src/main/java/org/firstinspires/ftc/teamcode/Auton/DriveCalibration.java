package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ButterflyDrive;
import org.firstinspires.ftc.teamcode.Subsystems.sdk.Action;
import org.firstinspires.ftc.teamcode.Subsystems.sdk.Drive;

@Config
@TeleOp(name = "Drive Calibration SDK", group = "Tuning")
public class DriveCalibration extends LinearOpMode {

    public static double TEST_DISTANCE_IN = 48;

    @Override
    public void runOpMode() {
        Telemetry dash = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drive drive = new Drive(hardwareMap, 0, 0, 90);
        ButterflyDrive butterfly = new ButterflyDrive(hardwareMap);
        butterfly.setMode(ButterflyDrive.Mode.MECANUM);

        Gamepad current = new Gamepad();
        Gamepad previous = new Gamepad();
        Action running = null;

        dash.addLine("Ready. Put the robot where it has room in front and to the right.");
        dash.update();

        waitForStart();
        if (isStopRequested()) return;
        drive.resetHeading();

        while (opModeIsActive()) {
            previous.copy(current);
            current.copy(gamepad1);

            if (running == null) {
                if (current.a && !previous.a) {
                    running = drive.moveTo(drive.getEstX(), drive.getEstY() + TEST_DISTANCE_IN, 90);
                } else if (current.b && !previous.b) {
                    running = drive.moveTo(drive.getEstX() + TEST_DISTANCE_IN, drive.getEstY(), 90);
                } else if (current.x && !previous.x) {
                    running = drive.turnTo(180);
                } else if (current.y && !previous.y) {
                    running = drive.turnTo(90);
                }
                if (running != null) {
                    running.start();
                }
            }

            if (running != null && running.update()) {
                running = null;
            }

            dash.addLine("A = forward TEST_DISTANCE_IN   B = strafe right TEST_DISTANCE_IN");
            dash.addLine("X = turn to 180 (left 90)   Y = turn back to 90");
            dash.addLine("Measure how far it really went, then in Configuration -> Drive:");
            dash.addLine("  FORWARD_IPS_PER_POWER = old x measured / TEST_DISTANCE_IN");
            dash.addLine("  STRAFE_IPS_PER_POWER  = old x measured / TEST_DISTANCE_IN");
            dash.addData("FORWARD_IPS_PER_POWER", "%.2f", Drive.FORWARD_IPS_PER_POWER);
            dash.addData("STRAFE_IPS_PER_POWER", "%.2f", Drive.STRAFE_IPS_PER_POWER);
            drive.addTelemetry(dash);
            dash.update();
        }

        if (running != null) {
            running.stop();
        }
        drive.stopMotors();
    }
}
