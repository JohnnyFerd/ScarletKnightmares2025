
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="AprilTag Calibration Test", group="Test")
public class AprilTagOpMode extends LinearOpMode {
    private AprilTag aprilTagSubsystem;
    private FtcDashboard dashboard;

    // Known distance you place your tag at (inches)
    private static final double KNOWN_DISTANCE_IN = 24.0;
    // Tag size (should match subsystem)
    private static final double TAG_SIZE_IN = 6.5;

    @Override
    public void runOpMode() {
        aprilTagSubsystem = new AprilTag(hardwareMap, telemetry);
        dashboard = FtcDashboard.getInstance();

        // Mirror telemetry to both DS phone + Dashboard
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            aprilTagSubsystem.update();
            aprilTagSubsystem.addTelemetry();

            // Extra tuning info
            AprilTagDetection detection = aprilTagSubsystem.getLatestTag();
            if (detection != null) {
                // Pixel width using first two corners (top edge)
                double pixelWidth = Math.abs(detection.corners[0].x - detection.corners[1].x);

                // Calculate focal length based on known distance
                double distance = (TAG_SIZE_IN * 827.0) / pixelWidth;
                telemetry.addData("Calculated Distance (in)", "%.2f", distance);

                telemetry.addData("Pixel Width (px)", "%.2f", pixelWidth);
                telemetry.addData("Known Distance (in)", KNOWN_DISTANCE_IN);
                telemetry.update();
            } else {
                telemetry.addLine("No tag detected");
            }

            telemetry.update();
        }

        aprilTagSubsystem.stop();
    }
}
