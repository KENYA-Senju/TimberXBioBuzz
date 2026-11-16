package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.Constants_New;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Studio Pedro Autonomous")
public class StudioPedroAuto extends OpMode {
    private Follower follower;

    // Use degrees for heading
    private final PoseFactory p = PoseFactory.degrees();

    // Define important positions
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose scorePose = p.of(48, 48, 90);
    private final Pose parkPose = p.of(72, 48, 90);

    // Define paths
    private Path startToScore() {
        return line(startPose, scorePose)
                .linear(startPose, scorePose);
    }

    private Path scoreToPark() {
        return line(scorePose, parkPose)
                .linear(scorePose, parkPose);
    }

    // Construct autonomous routine
    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),

                // Insert scoring mechanism command here

                follow(follower, scoreToPark())
        );
    }

    @Override
    public void init() {
        //Initialize Mechanism Motor


        Scheduler.reset();

        follower = Constants_New.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );
        telemetry.update();
    }
}
