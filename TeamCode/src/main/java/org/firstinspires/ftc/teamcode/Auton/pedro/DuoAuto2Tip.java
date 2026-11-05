package org.firstinspires.ftc.teamcode.Auton.pedro;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DuoAuto2Tip", group = "Autonomous")
public class DuoAuto2Tip extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.4938, 133.3025, 90);
    private final Pose move1 = poseFactory.of(23.4131, 97.2955, 270);
    private final Pose move1Control1 = poseFactory.of(54.6718, 103.4507, 0);
    private final Pose move2 = poseFactory.of(8.0038, 27.2459, 270);
    private final Pose move2Control1 = poseFactory.of(27.1159, 48.4157, 0);
    private final Pose intake1 = poseFactory.of(7.9272, 9.0896, 270);
    private final Pose move3 = poseFactory.of(35.0225, 108.3149, 270);
    private final Pose shoot2 = poseFactory.of(59.4938, 133.3025, 270);
    private final Pose shoot2Control1 = poseFactory.of(57.222, 109.4525, 0);
    private final Pose park = poseFactory.of(13.137, 95.2728, 270);
    private final Pose parkControl1 = poseFactory.of(58.672, 102.5328, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, move1()),
                follow(follower, move2()),
                follow(follower, intake1()),
                follow(follower, move3()),
                follow(follower, shoot2()),
                follow(follower, park())
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
        return Paths.curve(start, move1Control1, move1).constant(move1);
    }

    public Path move2() {
        return Paths.curve(move1, move2Control1, move2).constant(move2);
    }

    public Path intake1() {
        return Paths.line(move2, intake1).constant(intake1);
    }

    public Path move3() {
        return Paths.line(intake1, move3).constant(move3);
    }

    public Path shoot2() {
        return Paths.curve(move3, shoot2Control1, shoot2).constant(shoot2);
    }

    public Path park() {
        return Paths.curve(shoot2, parkControl1, park).constant(park);
    }
}
