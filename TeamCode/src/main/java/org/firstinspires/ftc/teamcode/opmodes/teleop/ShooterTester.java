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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "ShooterOpModePID")
@Config
public class ShooterTester extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    // === Dashboard Configurable Variables ===
    public static double DEGREE_OFFSET = 7.5; // adjustable on Dashboard
    public static double FOV_HORIZONTAL = 60.0; // camera FOV
    public static double TARGET_DISTANCE = 140.0; // inches from AprilTag to robot center
    public static double CENTER_TOLERANCE = .5; // pixels allowed off center
    public static double DISTANCE_TOLERANCE = 1.0; // inches allowed off target
    public static double CAMERA_LATERAL_OFFSET = -6.5; // inches, right is positive
    public static double CAMERA_YAW_OFFSET_DEG = -4.0; // degrees, camera points slightly right
    public static double MAX_ROTATE_SPEED = 0.25;
    public static double MAX_DRIVE_SPEED = 0.5;

    // PID coefficients (tune in dashboard)
    //TODO TUNE
    public static double kP_rotate = 0.0025;
    public static double kI_rotate = 0;
    public static double kD_rotate = 0;
    public static double kP_drive = -0.025;
    public static double kI_drive = 0;
    public static double kD_drive = 0;

    private AprilTag aprilTag;
    private JVBoysSoccerRobot robot;

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    private boolean shooterActive = false;

    private ElapsedTime timer = new ElapsedTime();

    // PID state
    private double errorXPrev = 0;
    private double distancePrev = 0;
    private double integralRotate = 0;
    private double integralDrive = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        aprilTag = new AprilTag(hardwareMap, telemetry);
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        RobotSettings.SUPER_TIME.reset();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            aprilTag.update(); // update AprilTag once per loop

            drivetrainControls(); // always drive
            shooterControl();     // shooter toggle
            lineupCorrection();   // adds correction if left bumper held

            robot.update(true, true);
        }
    }

    // === AprilTag correction overlay (PID) ===
    public void lineupCorrection() {
        if (!currentGamepad1.left_bumper) return; // only apply when held

        AprilTagDetection detection = aprilTag.getLatestTag();
        if (detection == null) {
            telemetry.addLine("No tag detected");
            return;
        }

        double errorX = detection.center.x - (aprilTag.getImageWidth() / 2.0);
        double distanceInches = aprilTag.getDistanceInches(detection);

        // Convert offset compensation
        double pixelsPerInch = aprilTag.getImageWidth() / (2 * TARGET_DISTANCE);
        errorX -= CAMERA_LATERAL_OFFSET * pixelsPerInch;
        double yawOffsetPixels = Math.tan(Math.toRadians(CAMERA_YAW_OFFSET_DEG)) * distanceInches * pixelsPerInch;
        errorX -= yawOffsetPixels;

        // Convert degree offset to pixel offset based on FOV
        double pixelOffset = (aprilTag.getImageWidth() / 2.0) *
                (Math.tan(Math.toRadians(DEGREE_OFFSET)) / Math.tan(Math.toRadians(FOV_HORIZONTAL / 2.0)));

        // Apply left/right rotation bias
        errorX -= pixelOffset;

        // Time step
        double dt = Math.max(0.001, timer.seconds());
        timer.reset();

        // === ROTATION PID ===
        double errorRotate = errorX;
        integralRotate += errorRotate * dt;
        double derivativeRotate = (errorRotate - errorXPrev) / dt;
        double rotateCorrection = kP_rotate * errorRotate + kI_rotate * integralRotate + kD_rotate * derivativeRotate;
        errorXPrev = errorRotate;

        // === DRIVE PID ===
        double errorDrive = distanceInches - TARGET_DISTANCE;
        integralDrive += errorDrive * dt;
        double derivativeDrive = (errorDrive - distancePrev) / dt;
        double forwardCorrection = kP_drive * errorDrive + kI_drive * integralDrive + kD_drive * derivativeDrive;
        distancePrev = errorDrive;

        // Dead zones
        if (Math.abs(errorRotate) < CENTER_TOLERANCE) rotateCorrection = 0;
        if (Math.abs(errorDrive) < DISTANCE_TOLERANCE) forwardCorrection = 0;

        // Limit speeds
        rotateCorrection = Math.max(-MAX_ROTATE_SPEED, Math.min(MAX_ROTATE_SPEED, rotateCorrection));
        forwardCorrection = Math.max(-MAX_DRIVE_SPEED, Math.min(MAX_DRIVE_SPEED, forwardCorrection));

        robot.drivetrainSubsystem.moveXYR(0, forwardCorrection, rotateCorrection);

        telemetry.addData("Tag ID", detection.id);
        telemetry.addData("X Error (px)", "%.2f", errorRotate);
        telemetry.addData("Distance (in)", "%.2f", distanceInches);
        telemetry.addData("Rotate PID", "%.3f", rotateCorrection);
        telemetry.addData("Forward PID", "%.3f", forwardCorrection);
    }

    // === Manual Drivetrain Controls ===
    public void drivetrainControls() {

        double x = gamepad1.left_stick_x * 1.05;
        double y = -gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            robot.drivetrainSubsystem.resetInitYaw();
        }

        if (currentGamepad1.right_trigger > 0.01 && currentGamepad1.left_trigger > 0.01) {
            x *= 0.3; y *= 0.3; r *= 0.3;
        } else if (currentGamepad1.right_trigger > 0.01 || currentGamepad1.left_trigger > 0.01) {
            x *= 0.65; y *= 0.65; r *= 0.65;
        }

        if (!currentGamepad1.left_bumper) {robot.drivetrainSubsystem.moveXYR(x, y, r);}

    }

    // === Shooter Controls ===
    public void shooterControl() {
        if (currentGamepad1.a && !previousGamepad1.a) {
            robot.shooterSubsystem.togglePaddle();
        }
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            shooterActive = !shooterActive;
            robot.shooterSubsystem.setVelocity(shooterActive ? 1 : 0);
        }
    }
}
