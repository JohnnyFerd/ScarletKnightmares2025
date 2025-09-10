package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="AprilTag ApproachV2", group="TeleOp")
public class AprilTagAngleApproach extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private AprilTag aprilTag;

    public static final double CENTER_TOLERANCE = 20;   // pixels off center allowed
    public static final double DISTANCE_TOLERANCE = 1.0; // inches tolerance
    public static final double TARGET_DISTANCE = 15.0;   // desired distance from tag

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        aprilTag = new AprilTag(hardwareMap, telemetry);

        telemetry.addLine("Ready to start AprilTag Approach");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            aprilTag.update();
            AprilTagDetection detection = aprilTag.getLatestTag();

            if (detection != null) {
                // Horizontal offset from image center
                double errorX = detection.center.x - (aprilTag.getImageWidth() / 2.0);
                // Distance in inches from camera to tag
                double distanceInches = aprilTag.getDistanceInches(detection);

                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("X Error (px)", "%.2f", errorX);
                telemetry.addData("Distance (in)", "%.2f", distanceInches);

                double forward = 0;
                double rotate = 0;

                // Step 1: Rotate to face tag
                if (Math.abs(errorX) > CENTER_TOLERANCE) {
                    rotate = (errorX > 0) ? 0.15 : -0.15; // adjust speed as needed
                }

                // Step 2: Move forward/backward to target distance
                if (Math.abs(distanceInches - TARGET_DISTANCE) > DISTANCE_TOLERANCE) {
                    forward = (distanceInches > TARGET_DISTANCE) ? 0.5 : -0.5;
                }

                // Drive forward/back with rotation correction
                robot.drivetrainSubsystem.moveXYR(0, -forward, rotate);

            } else {
                telemetry.addLine("No tag detected");
                robot.drivetrainSubsystem.moveXYR(0, 0, 0);
            }

            telemetry.update();
        }

        aprilTag.stop();
    }
}
