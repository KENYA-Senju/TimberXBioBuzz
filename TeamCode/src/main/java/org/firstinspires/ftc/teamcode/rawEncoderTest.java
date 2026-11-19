
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Pinpoint Telemetry", group = "Test")
public class rawEncoderTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;


    @Override
    public void runOpMode() {

        // Initialize Pinpoint
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // Set the encoder directions (adjust if necessary)
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // Initialize the Pinpoint
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Pinpoint Initialized");
        telemetry.addLine("Waiting for Start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update Pinpoint data
            pinpoint.update();

            // Read raw encoder ticks
            int xTicks = pinpoint.getEncoderX();
            int yTicks = pinpoint.getEncoderY();

            telemetry.addData("X Encoder Ticks", xTicks);
            telemetry.addData("Y Encoder Ticks", yTicks);

            // Additional diagnostic information
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.addData("X Position (mm)", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y Position (mm)", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)",
                    Math.toDegrees(pinpoint.getHeading(AngleUnit.DEGREES)));

            telemetry.update();

            sleep(20);
        }
    }
}
