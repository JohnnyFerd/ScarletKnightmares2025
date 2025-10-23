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
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;  // ✅ import your subsystem

@Config
@TeleOp(name = "Drive =", group = "FINAL")
public class TwoDrivernew extends LinearOpMode {

    private HardwareMap hwMap;
    private JVBoysSoccerRobot robot;
    private AprilTag aprilTag;  // ✅ declare AprilTag subsystem

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;

    private double previousX = 0, previousY = 0, previousR = 0;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private int loopCounter = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotSettings.SUPER_TIME.reset();

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        hwMap = hardwareMap;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hwMap, telemetry);
        aprilTag = new AprilTag(hwMap, telemetry); // ✅ initialize AprilTag subsystem

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.clear();
            RobotSettings.SUPER_TIME.reset();
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                drivetrainControls();
                loopTimes();

                robot.update(true, true);

                // ✅ Update and show AprilTag telemetry
                aprilTag.update();
                aprilTag.addTelemetry();

                // Optional: example action based on detection
                if (!aprilTag.goalLabel.equals("None")) {
                    telemetry.addData("Detected Tag", aprilTag.goalLabel);
                }

                telemetry.update();
            }
        }

        // ✅ Stop camera safely
        aprilTag.stop();
    }

    public void loopTimes() {
        loopCounter++;
        telemetry.addData("Loop Time", elapsedTime.milliseconds());
        telemetry.addData("Average Loop Time", RobotSettings.SUPER_TIME.milliseconds() / loopCounter);
        elapsedTime.reset();
    }

    public void drivetrainControls() {
        double x = gamepad1.left_stick_x * 1.05;
        double y = gamepad1.left_stick_y * -1;
        double r = gamepad1.right_stick_x;

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            robot.drivetrainSubsystem.resetInitYaw();
        }

        if (currentGamepad1.right_trigger > 0.01 && currentGamepad1.left_trigger > 0.01) {
            x *= 0.3;
            y *= 0.3;
            r *= 0.3;
        } else if (currentGamepad1.right_trigger > 0.01 || currentGamepad1.left_trigger > 0.01) {
            x *= 0.65;
            y *= 0.65;
            r *= 0.65;
        }

        if (!(previousX == x && previousY == y && previousR == r)) {
            robot.drivetrainSubsystem.moveXYR(x, y, r);
        }

        previousX = x;
        previousY = y;
        previousR = r;
    }
}
