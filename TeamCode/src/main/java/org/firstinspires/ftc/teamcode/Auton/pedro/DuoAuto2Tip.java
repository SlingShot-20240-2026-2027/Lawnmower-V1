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
import org.firstinspires.ftc.teamcode.Subsystems.DeployRamp;


@Autonomous(name = "DuoAuto2Tip", group = "Autonomous")
public class DuoAuto2Tip extends LinearOpMode {

    private Follower follower;
    private Shooternf shooter;
    private Transfer transfer;
    private Intake intake;
    private DeployRamp ramp;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.9698, 9.1683, 90);
    private final Pose move1 = poseFactory.of(9.5662, 9.6402, -92.5584);
    private final Pose move1Control1 = poseFactory.of(55.9677, 42.3578, 0);
    private final Pose move1Control2 = poseFactory.of(11.0775, 43.4643, 0);
    private final Pose point2 = poseFactory.of(16.6809, 22.7161, 267);
    private final Pose point3 = poseFactory.of(60.543, 118.911, -172.7707);
    private final Pose point3Control1 = poseFactory.of(20.9232, 113.8852, 0);
    private final Pose point4 = poseFactory.of(60.5309, 132.4671, -89.9492);
    private final Pose point5 = poseFactory.of(47.2105, 111.5869, 270);
    private final Pose point5Control1 = poseFactory.of(59.4066, 116.0037, 0);
    private final Pose point6 = poseFactory.of(47.261, 126.1652, 89.8016);
    private final Pose point7 = poseFactory.of(59.9794, 118.63, 90);
    private final Pose point8 = poseFactory.of(60.3762, 131.6093, -91.7513);
    private final Pose point9 = poseFactory.of(22.2281, 96.676, 270);
    private final Pose point9Control1 = poseFactory.of(44.8621, 102.6157, 0);
    private final Pose point10 = poseFactory.of(25.1049, 47.3061, -86.6651);
    private final Pose point11 = poseFactory.of(13.4245, 47.2274, -179.6141);
    private final Pose point12 = poseFactory.of(12.966, 96.2945, 180);
    private final Pose point12Control1 = poseFactory.of(32.6577, 73.5992, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                parallel(
                        shooter.on(),
                        intake.in()
                ),
                waitMs(1000),
                transfer.transfer(),
                waitMs(1000),
                transfer.idle(),
                follow(follower, move1()),
                waitMs(1000),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                transfer.transfer(),
                waitMs(1000),
                transfer.idle(),
                follow(follower, path5()),
                ramp.deploy(),
                follow(follower, path6()),
                ramp.up(),
                waitMs(1000),
                follow(follower, path7()),
                follow(follower, path8()),
                transfer.transfer(),
                waitMs(1000),
                transfer.idle(),
                follow(follower, path9()),
                follow(follower, path10()),
                ramp.deploy(),
                follow(follower, path11()),
                follow(follower, path12()),
                ramp.up(),
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
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
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
        return Paths.curve(start, move1Control1, move1Control2, move1).tangent();
    }

    public Path path2() {
        return Paths.line(move1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).reverseTangent();
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5).constant(point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }

    public Path path7() {
        return Paths.line(point6, point7).constant(point7);
    }

    public Path path8() {
        return Paths.line(point7, point8).reverseTangent();
    }

    public Path path9() {
        return Paths.curve(point8, point9Control1, point9).constant(point9);
    }

    public Path path10() {
        return Paths.line(point9, point10).tangent();
    }

    public Path path11() {
        return Paths.line(point10, point11).tangent();
    }

    public Path path12() {
        return Paths.curve(point11, point12Control1, point12).constant(point12);
    }
}
