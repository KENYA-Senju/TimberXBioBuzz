package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants_New {

    // 1. Drivetrain Motors Configuration
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    // 2. goBILDA Pinpoint Configuration
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0019195124389618401);
        c.yPodOffset.set(-0.010796460345035463);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    // 3. Foresight Configuration (Baseline defaults for compilation)
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // Cross-track correction for a strafe path runs through the robot's forward
                // axis, so this gain must be modest or it saturates the allocator before the
                // strafe drive vector gets any power. Pedro's reference config uses ~0.3.
                Controller primaryTranslationalForward = Controller.proportional(0.5048691243902689);
                Controller secondaryTranslationalForward = Controller.proportional(0.18653565111074524);
                Controller primaryTranslationalLateral = Controller.proportional(0.5048691243902689);
                Controller secondaryTranslationalLateral = Controller.proportional(0.18653565111074524);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.02130450038616258));
                c.brake.set(Controller.proportionalFeedforward(0.018108825328238196));

                c.headingFeedback.set(Controller.proportional(5.707171768433237));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03504659068145582, 0.01064077257434124));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06890407560783118, 0.04544335295071627));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016625149344513931, 0.0018988939062750381));

                c.maxAchievableForwardVelocity.set(53.57812916260591);
                c.maxAchievableStrafeVelocity.set(40.76310911547647);
                c.naturalForwardDeceleration.set(35.40216597184375);
                c.naturalStrafeDeceleration.set(72.02921486095074);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
