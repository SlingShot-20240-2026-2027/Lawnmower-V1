package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.shooter.Turret;

import java.util.List;

@TeleOp
public class TurretTestNew extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Turret turret = new Turret(hardwareMap);
        double target = 0;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            if (gamepad1.aWasPressed()) target = Math.toRadians(45);
            if (gamepad1.bWasPressed()) target = Math.toRadians(90);
            if (gamepad1.xWasPressed()) target = Math.toRadians(-45);
            if (gamepad1.yWasPressed()) target = Math.toRadians(-90);
            if (gamepad1.dpadDownWasPressed()) target = 0;
            if (gamepad1.dpadLeftWasPressed()) target += Math.toRadians(gamepad1.left_bumper ? 1 : 10);
            if (gamepad1.dpadRightWasPressed()) target -= Math.toRadians(gamepad1.left_bumper ? 1 : 10);

            turret.setTurretAngle(target);
            turret.update();

            telemetry.addData("Target (deg)", Math.toDegrees(turret.getTargetAngle()));
            telemetry.addData("Current (deg)", Math.toDegrees(turret.getCurrentAngle()));
            telemetry.addData("Error (deg)", Math.toDegrees(turret.getTargetAngle() - turret.getCurrentAngle()));
            telemetry.update();
        }
        turret.stop();
    }
}
