package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.shooter.Turret;

@TeleOp
public class TurretCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Turret turret = new Turret(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double p = -gamepad1.left_stick_x * 0.3;

            double left = gamepad1.right_bumper ? 0 : p;
            double right = gamepad1.left_bumper ? 0 : p;
            turret.setRawPower(left, right);

            telemetry.addData("Raw encoder (V)", turret.getRawEncoderVoltage());
            telemetry.addData("Raw encoder (deg)", turret.getRawEncoderDegrees());
            telemetry.addData("Turret angle (deg)", Math.toDegrees(turret.getCurrentAngle()));
            telemetry.addData("Left power", left);
            telemetry.addData("Right power", right);
            telemetry.update();
        }
        turret.stop();
    }
}
