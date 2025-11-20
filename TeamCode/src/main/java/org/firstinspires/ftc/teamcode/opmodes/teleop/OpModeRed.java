package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "OpModeRed")
public class OpModeRed extends LinearOpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // === Shooter Alignment / PID Tunables ===
    public static double DEGREE_OFFSET = 12;
    public static double TARGET_DISTANCE = 140.0;
    public static double CENTER_TOLERANCE = 0.5;
    public static double DISTANCE_TOLERANCE = 1.0;

    public static double kP_rotate = 0.0025, kI_rotate = 0, kD_rotate = 0;
    public static double kP_drive = -0.025, kI_drive = 0, kD_drive = 0;
    public static double MAX_ROTATE_SPEED = 0.25;
    public static double MAX_DRIVE_SPEED = 0.5;

    // === Subsystems ===
    private JVBoysSoccerRobot robot;
    private AprilTag aprilTag;

    // === Gamepad State ===
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    // === PID Tracking ===
    private double errorXPrev = 0;
    private double distancePrev = 0;
    private double integralRotate = 0;
    private double integralDrive = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // === Shooter Sequence Variables ===
    private boolean sequenceActive = false;
    private int sequenceStep = 0;
    private double sequenceTimer = 0;
    private boolean shooterActive = false;

    // === Configurable delays for each step (dashboard adjustable) ===
    public static double DELAY_0_UP_TO_DOWN = 0.5;
    public static double DELAY_1_DOWN_TO_UP = 0.45;
    public static double DELAY_2_UP_TO_DOWN = 0.75;
    public static double DELAY_3_DOWN_TO_UP = 0.6;
    public static double DELAY_4_UPLAST_TO_DOWN = 0.8;
    public static double DELAY_5_DOWN_TO_START = 0.5;

    // === AprilTag Handling ===
    private AprilTagDetection lastDetection = null;
    private final ElapsedTime lastDetectionTimer = new ElapsedTime();
    public static double TAG_HOLD_TIME = 0.1; // seconds to keep last known detection
    public static int TARGET_TAG_ID = 21;     // only track this tag
    private boolean usingAprilTagAlignment = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        aprilTag = robot.aprilTag;

        telemetry.addLine("Initialized - Ready to Start");
        telemetry.update();

        waitForStart();
        RobotSettings.SUPER_TIME.reset();
        timer.reset();

        while (opModeIsActive()) {
            updateGamepadStates();

            handleAprilTagCorrection();
            handleDrivetrainControls();
            handleShooterControls();

            robot.update(true, true);
            telemetry.update();
        }
    }

    private void handleAprilTagCorrection() {
        if (!currentGamepad1.left_bumper) {
            usingAprilTagAlignment = false;
            return;
        }

        AprilTagDetection detection = aprilTag.getLatestTag();

        if (detection != null && detection.id == TARGET_TAG_ID) {
            lastDetection = detection;
            lastDetectionTimer.reset();
        } else if (lastDetection != null && lastDetection.id == TARGET_TAG_ID && lastDetectionTimer.seconds() < TAG_HOLD_TIME) {
            detection = lastDetection;
        } else {
            usingAprilTagAlignment = false;
            return;
        }

        usingAprilTagAlignment = true;

        double errorX = detection.center.x - (aprilTag.getImageWidth() / 2.0);
        double distanceInches = aprilTag.getDistanceInches(detection);

        double pixelsPerInch = aprilTag.getImageWidth() / (2 * TARGET_DISTANCE);
        errorX -= aprilTag.CAMERA_LATERAL_OFFSET * pixelsPerInch;
        errorX -= Math.tan(Math.toRadians(aprilTag.CAMERA_YAW_OFFSET_DEG)) * distanceInches * pixelsPerInch;
        errorX -= (aprilTag.getImageWidth() / 2.0) *
                (Math.tan(Math.toRadians(DEGREE_OFFSET)) / Math.tan(Math.toRadians(aprilTag.CAMERA_FOV_DEG / 2.0)));

        double dt = Math.max(0.001, timer.seconds());
        timer.reset();

        // Rotation PID
        double errorRotate = errorX;
        integralRotate += errorRotate * dt;
        double derivativeRotate = (errorRotate - errorXPrev) / dt;
        double rotateCorrection = kP_rotate * errorRotate + kI_rotate * integralRotate + kD_rotate * derivativeRotate;
        errorXPrev = errorRotate;

        // Drive PID
        double errorDrive = distanceInches - TARGET_DISTANCE;
        integralDrive += errorDrive * dt;
        double derivativeDrive = (errorDrive - distancePrev) / dt;
        double forwardCorrection = kP_drive * errorDrive + kI_drive * integralDrive + kD_drive * derivativeDrive;
        distancePrev = errorDrive;

        if (Math.abs(errorRotate) < CENTER_TOLERANCE) rotateCorrection = 0;
        if (Math.abs(errorDrive) < DISTANCE_TOLERANCE) forwardCorrection = 0;

        rotateCorrection = clamp(rotateCorrection, -MAX_ROTATE_SPEED, MAX_ROTATE_SPEED);
        forwardCorrection = clamp(forwardCorrection, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

        robot.drivetrainSubsystem.moveXYR(0, forwardCorrection, rotateCorrection);

        telemetry.addData("Tracking Tag", TARGET_TAG_ID);
        telemetry.addData("Detected Tag ID", detection.id);
        telemetry.addData("X Error (px)", "%.2f", errorRotate);
        telemetry.addData("Distance (in)", "%.2f", distanceInches);
        telemetry.addData("Rotate PID", "%.3f", rotateCorrection);
        telemetry.addData("Forward PID", "%.3f", forwardCorrection);
        telemetry.addData("Tag Hold Time", "%.2f s", lastDetectionTimer.seconds());
    }

    // === Manual Drivetrain Controls ===
    private void handleDrivetrainControls() {
        if (usingAprilTagAlignment) return;

        double x = -gamepad2.left_stick_x * 1.05;
        double y = gamepad2.left_stick_y;
        double r = gamepad2.right_stick_x;

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
            robot.drivetrainSubsystem.resetInitYaw();

        double speedScale = 1.0;
        if (currentGamepad2.right_trigger > 0.01 && currentGamepad2.left_trigger > 0.01) speedScale = 0.1;
        else if (currentGamepad2.right_trigger > 0.01 || currentGamepad2.left_trigger > 0.01) speedScale = 0.2;

        x *= speedScale;
        y *= speedScale;
        r *= speedScale;

        robot.drivetrainSubsystem.moveXYR(x, y, r);
    }

    // === Shooter Controls ===
    private void handleShooterControls() {
        // Reset sequence on X press
        if (currentGamepad1.x && !previousGamepad1.x) {
            sequenceActive = false;
            sequenceStep = 0;
            sequenceTimer = 0;
            robot.shooterSubsystem.paddleDown();
        }

        // Paddle Sequence (hold A)
        if (currentGamepad1.a) {
            if (!sequenceActive) {
                sequenceActive = true;
                sequenceStep = 0;
                sequenceTimer = 0;
            }

            double dt = timer.seconds();
            timer.reset();
            sequenceTimer += dt;

            double currentDelay = getCurrentStepDelay();

            if (sequenceTimer >= currentDelay) {
                sequenceTimer = 0;
                switch (sequenceStep) {
                    case 0: robot.shooterSubsystem.paddleUp(); break;
                    case 1: robot.shooterSubsystem.paddleDown(); break;
                    case 2: robot.shooterSubsystem.paddleUp(); break;
                    case 3: robot.shooterSubsystem.paddleDown(); break;
                    case 4: robot.shooterSubsystem.paddleUpLast(); break;
                    case 5: robot.shooterSubsystem.paddleDown(); break;
                }
                sequenceStep = (sequenceStep + 1) % 6;
            }
        } else {
            sequenceActive = false;
        }

        // Manual Paddle
        if (currentGamepad1.b && !previousGamepad1.b)
            robot.shooterSubsystem.paddleUp();

        // Toggle Shooter Flywheel
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            // If already at Far speed, turn off; otherwise switch to Far speed
            if (shooterActive && robot.shooterSubsystem.getVelocity() == Shooter.FarShotVelo) {
                shooterActive = false;
                robot.shooterSubsystem.setVelocity(0);
            } else {
                shooterActive = true;
                robot.shooterSubsystem.setVelocity(Shooter.FarShotVelo);
            }
        }

        else if (currentGamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger <= 0.1) {
            // If already at Close speed, turn off; otherwise switch to Close speed
            if (shooterActive && robot.shooterSubsystem.getVelocity() == Shooter.CloseShotVelo) {
                shooterActive = false;
                robot.shooterSubsystem.setVelocity(0);
            } else {
                shooterActive = true;
                robot.shooterSubsystem.setVelocity(Shooter.CloseShotVelo);
            }
        }
    }

    private double getCurrentStepDelay() {
        switch (sequenceStep) {
            case 0: return DELAY_0_UP_TO_DOWN;
            case 1: return DELAY_1_DOWN_TO_UP;
            case 2: return DELAY_2_UP_TO_DOWN;
            case 3: return DELAY_3_DOWN_TO_UP;
            case 4: return DELAY_4_UPLAST_TO_DOWN;
            case 5: return DELAY_5_DOWN_TO_START;
            default: return 0.5;
        }
    }

    private void updateGamepadStates() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
