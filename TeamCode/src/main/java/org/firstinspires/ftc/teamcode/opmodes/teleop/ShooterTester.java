package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "ShooterOpMode")
@Config
public class ShooterTester extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    // === Dashboard Configurable Variables ===
    public static double CENTER_TOLERANCE = 20.0;   // pixels off center allowed
    public static double DISTANCE_TOLERANCE = 1.0;  // inches tolerance
    public static double TARGET_DISTANCE = 15.0;    // desired distance from tag
    public static double ROTATE_SPEED = 0.15;       // rotation correction speed
    public static double DRIVE_SPEED = 0.5;         // forward/back correction speed
    public static double CAMERA_LATERAL_OFFSET = 0.0; // inches, right is positive

    private AprilTag aprilTag;
    private JVBoysSoccerRobot robot;

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    private boolean shooterActive = false;
    private double previousX = 0, previousY = 0, previousR = 0;

    private ElapsedTime timer = new ElapsedTime();
    private HardwareMap hwMap;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = hardwareMap;
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        aprilTag = new AprilTag(hardwareMap, telemetry);
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        RobotSettings.SUPER_TIME.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            RobotSettings.SUPER_TIME.reset();
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                lineupAuto();
                drivetrainControls();
                shooterControl();

                robot.update(true, true);
            }
        }
    }

    public void lineupAuto() {
        if (currentGamepad1.left_bumper) {
            aprilTag.update();
            AprilTagDetection detection = aprilTag.getLatestTag();

            if (detection != null) {
                double errorX = detection.center.x - (aprilTag.getImageWidth() / 2.0);

                // Adjust horizontal error for lateral camera offset
                double pixelsPerInch = aprilTag.getImageWidth() / (2 * TARGET_DISTANCE); // rough scaling
                errorX -= CAMERA_LATERAL_OFFSET * pixelsPerInch;

                double distanceInches = aprilTag.getDistanceInches(detection);

                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("X Error (px)", "%.2f", errorX);
                telemetry.addData("Distance (in)", "%.2f", distanceInches);

                double forward = 0;
                double rotate = 0;

                if (Math.abs(errorX) > CENTER_TOLERANCE) {
                    rotate = (errorX > 0) ? ROTATE_SPEED : -ROTATE_SPEED;
                }
                if (Math.abs(distanceInches - TARGET_DISTANCE) > DISTANCE_TOLERANCE) {
                    forward = (distanceInches > TARGET_DISTANCE) ? DRIVE_SPEED : -DRIVE_SPEED;
                }

                robot.drivetrainSubsystem.moveXYR(0, -forward, rotate);
            } else {
                telemetry.addLine("No tag detected");
                robot.drivetrainSubsystem.moveXYR(0, 0, 0);
            }
        }
    }

    public void drivetrainControls() {
        double x = gamepad1.left_stick_x * 1.05;
        double y = gamepad1.left_stick_y * -1;
        double r = gamepad1.right_stick_x;

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            robot.drivetrainSubsystem.resetInitYaw();
        }

        if (currentGamepad1.right_trigger > 0.01 && currentGamepad1.left_trigger > 0.01) {
            x *= 0.3; y *= 0.3; r *= 0.3;
        } else if (currentGamepad1.right_trigger > 0.01 || currentGamepad1.left_trigger > 0.01) {
            x *= 0.65; y *= 0.65; r *= 0.65;
        }
            robot.drivetrainSubsystem.moveXYR(x, y, r);


        previousX = x;
        previousY = y;
        previousR = r;
    }

    public void shooterControl() {
        if (currentGamepad1.a && !previousGamepad1.a) {
            robot.shooterSubsystem.togglePaddle();
        }
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            if (shooterActive) {
                robot.shooterSubsystem.setVelocity(0);
                shooterActive = false;
            } else {
                robot.shooterSubsystem.setVelocity(1);
                shooterActive = true;
            }
        }
    }
}
