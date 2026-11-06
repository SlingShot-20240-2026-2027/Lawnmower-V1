package org.firstinspires.ftc.teamcode.Auton.pedro;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooternf;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;


@Autonomous(name = "SoloAuto1Tip", group = "Autonomous")
public class SoloAuto1Tip extends LinearOpMode {

    private Follower follower;
    private Shooternf shooter;
    private Transfer transfer;
    private Intake intake;
    private Gate gate;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.91998822932052, 9.218138386262925, 90);
    private final Pose move1 = poseFactory.of(8.75, 29.3925, 270);
    private final Pose move1Control1 = poseFactory.of(49.2774, 16.9788, 0);
    private final Pose intake1 = poseFactory.of(8.75, 8.7776, 270);
    private final Pose park = poseFactory.of(13.6905, 95.7133, 270);
    private final Pose parkControl1 = poseFactory.of(40.64, 71.0928, 0);
//Eshaann
    // Autonomous routine
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
                follow(follower, move1()),
                follow(follower, intake1()),
                follow(follower, park()),
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
        follower = Constants.create(hardwareMap);
        shooter = new Shooternf(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(shooter.periodic());
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            intake.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path move1() {
        return Paths.curve(start, move1Control1, move1).linear(start, move1);
    }

    public Path intake1() {
        return Paths.line(move1, intake1).constant(intake1);
    }

    public Path park() {
        return Paths.curve(intake1, parkControl1, park).constant(park);
    }
}
