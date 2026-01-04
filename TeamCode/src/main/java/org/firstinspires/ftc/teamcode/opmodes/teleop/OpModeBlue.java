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
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.outake;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "OpModeBlue", group = "TeleOp")
public class OpModeBlue extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    /* ===== Subsystems ===== */
    private JVBoysSoccerRobot robot;
    private AprilTag aprilTag;
    private Spindexer spindexer;
    private outake outake;

    /* ===== Gamepad State ===== */
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    /* ===== Intake / Outtake State ===== */
    private boolean intakeOn = false;
    private boolean outtakeOn = false;

    private enum OuttakeMode { FAR, CLOSE }
    private OuttakeMode outtakeMode = OuttakeMode.FAR;

    /* ===== AprilTag ===== */
    private AprilTagDetection lastDetection = null;
    private final ElapsedTime lastDetectionTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        aprilTag = robot.aprilTag;
        spindexer = robot.spindexer;
        outake = robot.outake;

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        RobotSettings.SUPER_TIME.reset();

        while (opModeIsActive()) {
            updateGamepadStates();

            handleDrivetrainControls();
            handleShooterControls();

            // ===== Update subsystems every loop =====
            robot.spindexer.update();
            robot.outake.update();
            robot.intake.update();

            robot.addTelemetry();
            telemetry.update();
        }

    }

    /* ===== DRIVETRAIN (GAMEPAD 1) ===== */
    private void handleDrivetrainControls() {

        double speedScale = 1.0;

        boolean slow = currentGamepad1.left_bumper;
        boolean slower = currentGamepad1.left_trigger > 0.2;

        if (slow && slower) speedScale = 0.3;
        else if (slow || slower) speedScale = 0.6;

        double x = gamepad1.left_stick_x * 1.05 * speedScale;
        double y = gamepad1.left_stick_y * speedScale;
        double r = gamepad1.right_stick_x * speedScale;

        robot.drivetrainSubsystem.moveXYR(x, -y, r);
    }

    /* ===== SHOOTER / INTAKE / OUTTAKE (GAMEPAD 2) ===== */
    private void handleShooterControls() {

        /* --- SPINDEXER SHOOTING --- */

        if (currentGamepad2.a && !previousGamepad2.a) {
            spindexer.shoot();
        }
        if (currentGamepad2.b && !previousGamepad2.b) {
            spindexer.catalogue();
        }

        if (currentGamepad2.x && !previousGamepad2.x) {
            spindexer.shootPurple();
        }

        if (currentGamepad2.y && !previousGamepad2.y)
        {
            spindexer.shootGreen();
        }

        /* --- OUTTAKE --- */

        // Right Trigger → FAR velocity toggle
        if (currentGamepad2.right_trigger > 0.2 && previousGamepad2.right_trigger <= 0.2) {
            if (outtakeOn && outtakeMode == OuttakeMode.FAR) {
                outake.outakeOff();
                outtakeOn = false;
            } else {
                outake.setFarShot();
                outake.outakeOn();
                outtakeMode = OuttakeMode.FAR;
                outtakeOn = true;
            }
        }

        // Right Bumper → CLOSE velocity toggle
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            if (outtakeOn && outtakeMode == OuttakeMode.CLOSE) {
                outake.outakeOff();
                outtakeOn = false;
            } else {
                outake.setCloseShot();
                outake.outakeOn();
                outtakeMode = OuttakeMode.CLOSE;
                outtakeOn = true;
            }
        }

        /* --- INTAKE --- */
        if (currentGamepad2.left_trigger > 0.2 && previousGamepad2.left_trigger <= 0.2) {
            intakeOn = !intakeOn;
            if (intakeOn) robot.intake.intakeOn();
            else robot.intake.intakeOff();
        }
    }

    /* ===== GAMEPAD EDGE TRACKING ===== */
    private void updateGamepadStates() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }
}
