package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants_New;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "BioBuzz Red Right")
public class StudioPedroAuto extends OpMode {
    private static final PoseFactory POSES = PoseFactory.degrees();
    // Use the robot's starting position and forward direction as the local frame.
    private static final Pose START_POSE = POSES.of(0, 0, 0);
    private static final Pose END_POSE = POSES.of(10, 0, 0);
    private static final double TIMEOUT_SECONDS = 10.0;
    private static final double POSITION_TOLERANCE_INCHES = 0.5;
    private static final double SETTLE_SECONDS = 0.3;

    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();
    private Follower follower;
    private boolean driving;
    private String status = "Ready to drive forward 10 inches";

    @Override
    public void init() {
        follower = Constants_New.create(hardwareMap);
        follower.holdEnd.set(true);
        reportTelemetry();
    }

    @Override
    public void start() {
        follower.setPose(START_POSE);
        follower.follow(line(START_POSE, END_POSE).linear(START_POSE, END_POSE));
        moveTimer.reset();
        settleTimer.reset();
        driving = true;
        status = "Driving forward";
    }

    @Override
    public void loop() {
        if (driving) {
            if (moveTimer.seconds() >= TIMEOUT_SECONDS) {
                stop();
                status = "Drive timed out";
            } else {
                follower.update();
                Pose current = follower.pose();
                double distanceToEnd = Math.hypot(END_POSE.x() - current.x(),
                        END_POSE.y() - current.y());
                if (distanceToEnd > POSITION_TOLERANCE_INCHES) {
                    settleTimer.reset();
                } else if (settleTimer.seconds() >= SETTLE_SECONDS) {
                    stop();
                    status = "Drive complete";
                }
            }
        }
        reportTelemetry();
    }

    @Override
    public void stop() {
        driving = false;
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    private void reportTelemetry() {
        telemetry.addData("Status", status);
        telemetry.addData("Forward (in)", follower.pose().x());
        telemetry.addData("Lateral (in)", follower.pose().y());
        telemetry.update();
    }
}
