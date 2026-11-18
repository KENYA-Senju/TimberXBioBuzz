package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants_New;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "BioBuzz Red Right")
public class StudioPedroAuto extends OpMode {
    private static final double LEG_INCHES = 10.0;
    private static final double FULL_TURN_RADIANS = Math.PI;
    private static final double TURN_POWER = 0.6;
    private static final double MIN_TURN_POWER = 0.2;

    private static final double FORWARD_TIMEOUT_SECONDS = 8.0;
    private static final double RIGHT_TIMEOUT_SECONDS = 8.0;
    private static final double SPIN_TIMEOUT_SECONDS = 6.0;

    // Guard against a bad localizer reading.
    private static final double MAX_SPEED_IN_PER_SEC = 120.0;
    private static final double MAX_TURN_RATE_RAD_PER_SEC = 8.0;
    private static final double POSITION_MARGIN_IN = 0.75;
    private static final double HEADING_MARGIN_RAD = 0.35;

    private enum Step { FORWARD, RIGHT, SPIN, DONE }

    private final ElapsedTime stepTimer = new ElapsedTime();
    private Follower follower;
    private Path forwardPath;
    private Path rightPath;
    private Step step = Step.FORWARD;
    private Pose lastGoodPose;
    private long lastLoopNanos;
    private double previousHeading;
    private double turnAmount;
    private boolean finished;
    private String status = "Ready: forward 10 in, right 10 in, spin 180 deg";

    @Override
    public void init() {
        Pose start = Pose.zero();
        Pose afterForward = new Pose(LEG_INCHES, 0, 0);           // +x is forward
        Pose afterRight = new Pose(LEG_INCHES, -LEG_INCHES, 0);   // -y is right

        forwardPath = line(start, afterForward).constant(0);
        rightPath = line(afterForward, afterRight).constant(0);

        follower = Constants_New.create(hardwareMap);
        follower.holdEnd.set(false);
        follower.setPose(start);
        follower.update();

        lastGoodPose = follower.pose();
        lastLoopNanos = System.nanoTime();
        reportTelemetry(lastGoodPose);
    }

    @Override
    public void start() {
        beginStep(Step.FORWARD);
        follower.follow(forwardPath);
        status = "Driving forward 10 inches";
    }

    @Override
    public void loop() {
        if (finished) {
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastLoopNanos) / 1_000_000_000.0;
        lastLoopNanos = now;

        follower.update();
        Pose current = follower.pose();

        if (dt > 0 && implausible(current, dt)) {
            follower.setPose(lastGoodPose);
            finish("Aborted: localizer jump", current);
            return;
        }
        lastGoodPose = current;

        if (stepTimer.seconds() >= stepTimeoutSeconds()) {
            finish("Step timed out", current);
            return;
        }

        switch (step) {
            case FORWARD:
                if (follower.atParametricEnd()) {
                    beginStep(Step.RIGHT);
                    follower.follow(rightPath);
                    status = "Strafing right 10 inches";
                }
                break;

            case RIGHT:
                if (follower.atParametricEnd()) {
                    beginStep(Step.SPIN);
                    previousHeading = current.heading();
                    turnAmount = 0;
                    follower.manual(0, 0, TURN_POWER);
                    status = "Spinning 180 degrees";
                }
                break;

            case SPIN:
                double heading = current.heading();
                turnAmount += Angle.normalizeSigned(heading - previousHeading);
                previousHeading = heading;

                double remaining = FULL_TURN_RADIANS - Math.abs(turnAmount);
                if (remaining <= 0) {
                    finish("Routine complete", current);
                    return;
                }

                double power = Math.max(MIN_TURN_POWER,
                        Math.min(TURN_POWER, TURN_POWER * remaining / FULL_TURN_RADIANS));
                follower.manual(0, 0, power);
                break;

            case DONE:
                break;
        }

        reportTelemetry(current);
    }

    @Override
    public void stop() {
        stopDriving();
    }

    private void beginStep(Step next) {
        step = next;
        stepTimer.reset();
    }

    private double stepTimeoutSeconds() {
        switch (step) {
            case FORWARD: return FORWARD_TIMEOUT_SECONDS;
            case RIGHT: return RIGHT_TIMEOUT_SECONDS;
            case SPIN: return SPIN_TIMEOUT_SECONDS;
            default: return 0;
        }
    }

    private boolean implausible(Pose current, double dt) {
        double maxMove = MAX_SPEED_IN_PER_SEC * dt + POSITION_MARGIN_IN;
        double maxTurn = MAX_TURN_RATE_RAD_PER_SEC * dt + HEADING_MARGIN_RAD;
        return lastGoodPose != null
                && (current.distance(lastGoodPose) > maxMove
                || Math.abs(Angle.normalizeSigned(current.heading() - lastGoodPose.heading())) > maxTurn);
    }

    private void finish(String message, Pose pose) {
        status = message;
        step = Step.DONE;
        stopDriving();
        reportTelemetry(pose);
        finished = true;
        terminateOpModeNow();
    }

    private void stopDriving() {
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    private void reportTelemetry(Pose pose) {
        telemetry.addData("Status", status);
        telemetry.addData("Step", step);
        telemetry.addData("Forward (in)", pose.x());
        telemetry.addData("Right (in)", -pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("Turn (deg)", Math.toDegrees(turnAmount));
        telemetry.update();
    }
}
