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

@Autonomous(name = "AutoPath", group = "Autonomous")
public class MainPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.7018, 8.4678, 90);
    private final Pose move1 = poseFactory.of(8.0215, 29.3925, 270);
    private final Pose move1Control1 = poseFactory.of(49.2774, 16.9788, 0);
    private final Pose intake1 = poseFactory.of(8.02, 8.7776, 270);
    private final Pose move2 = poseFactory.of(38.8782, 115.5201, 270);
    private final Pose shoot2 = poseFactory.of(59.6324, 132.6809, 270);
    private final Pose shoot2Control1 = poseFactory.of(59.8531, 123.6612, 0);
    private final Pose move3Start = poseFactory.of(59.6324, 132.6809, 270);
    private final Pose move3 = poseFactory.of(46.7537, 110.4212, 90);
    private final Pose intake2 = poseFactory.of(46.7897, 126.5176, 90);
    private final Pose move4 = poseFactory.of(60.1102, 112.5385, 90);
    private final Pose shoot3Start = poseFactory.of(60.1102, 112.5385, 90);
    private final Pose shoot3 = poseFactory.of(60.0907, 132.063, 270);
    private final Pose move5Start = poseFactory.of(60.0907, 132.063, 270);
    private final Pose move5 = poseFactory.of(22.6, 46.6465, 180);
    private final Pose move5Control1 = poseFactory.of(19.204, 81.6708, 0);
    private final Pose intake3 = poseFactory.of(13.6743, 46.6829, 180);
    private final Pose park = poseFactory.of(11.1673, 96.3131, 180);
    private final Pose parkControl1 = poseFactory.of(29.2621, 68.8452, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, move1()),
                follow(follower, intake1()),
                follow(follower, move2()),
                follow(follower, shoot2()),
                follow(follower, move3()),
                follow(follower, intake2()),
                follow(follower, move4()),
                follow(follower, shoot3()),
                follow(follower, move5()),
                follow(follower, intake3()),
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
        return Paths.curve(start, move1Control1, move1).linear(start, move1);
    }

    public Path intake1() {
        return Paths.line(move1, intake1).constant(intake1);
    }

    public Path move2() {
        return Paths.line(intake1, move2).constant(move2);
    }

    public Path shoot2() {
        return Paths.curve(move2, shoot2Control1, shoot2).constant(shoot2);
    }

    public Path move3() {
        return Paths.line(move3Start, move3).linear(move3Start, move3);
    }

    public Path intake2() {
        return Paths.line(move3, intake2).constant(intake2);
    }

    public Path move4() {
        return Paths.line(intake2, move4).constant(move4);
    }

    public Path shoot3() {
        return Paths.line(shoot3Start, shoot3).linear(shoot3Start, shoot3);
    }

    public Path move5() {
        return Paths.curve(move5Start, move5Control1, move5).linear(move5Start, move5);
    }

    public Path intake3() {
        return Paths.line(move5, intake3).constant(intake3);
    }

    public Path park() {
        return Paths.curve(intake3, parkControl1, park).constant(park);
    }
}
