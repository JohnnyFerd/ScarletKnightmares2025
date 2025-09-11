package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="AprilTag Approach", group="TeleOp")
public class AprilTagApproach extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private AprilTag aprilTag;
    private FtcDashboard dashboard;

    public static final double CENTER_TOLERANCE = 20;   // pixels off center allowed
    public static final double DISTANCE_TOLERANCE = 1.0; // inches tolerance
    public static final double TARGET_DISTANCE = 15.0;   // desired distance from tag

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        aprilTag = new AprilTag(hardwareMap, telemetry);
        dashboard = FtcDashboard.getInstance();

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

                double strafe = 0;
                double forward = 0;

                // Step 1: Center the tag
                if (Math.abs(errorX) > CENTER_TOLERANCE) {
                    strafe = (errorX > 0) ? 0.2 : -0.2; // strafe left/right
                }

                // Step 2: Approach or back up to target distance
                if (Math.abs(distanceInches - TARGET_DISTANCE) > DISTANCE_TOLERANCE) {
                    forward = (distanceInches > TARGET_DISTANCE) ? 0.25 : -0.25;
                }

                // Drive with XY + no rotation
                robot.drivetrainSubsystem.moveXYR(strafe, -forward, 0);

            } else {
                telemetry.addLine("No tag detected");
                robot.drivetrainSubsystem.moveXYR(0, 0, 0);
            }

            telemetry.update();
        }

        aprilTag.stop();
    }
}
