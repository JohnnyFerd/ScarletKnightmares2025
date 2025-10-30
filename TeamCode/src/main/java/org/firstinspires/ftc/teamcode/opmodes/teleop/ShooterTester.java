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
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "ShooterOpMode")
public class ShooterTester extends LinearOpMode {
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    ElapsedTime timer = new ElapsedTime();
    HardwareMap hwMap;
    private double previousX = 0, previousY = 0, previousR = 0;
    JVBoysSoccerRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = hardwareMap;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        RobotSettings.SUPER_TIME.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            telemetry.clear();
            RobotSettings.SUPER_TIME.reset();
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                drivetrainControls();
                shooterControl();
                robot.update(true, true);
            }
        }
    }

    public void drivetrainControls() {
        double x = gamepad1.left_stick_x * 1.05;
        double y = gamepad1.left_stick_y * -1;
        double r = gamepad1.right_stick_x;

//        if (currentGamepad1.b && !previousGamepad1.b) {
//            robot.drivetrainSubsystem.isFieldCentric = !robot.drivetrainSubsystem.isFieldCentric;
//        }

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            robot.drivetrainSubsystem.resetInitYaw();
        }

//        if (currentGamepad1.b && !previousGamepad1.b) {
//            robot.drivetrainSubsystem.orthogonalMode = !robot.drivetrainSubsystem.orthogonalMode;
//        }

        if (currentGamepad1.right_trigger > 0.01 && currentGamepad1.left_trigger > 0.01) {
            x *= 0.3;
            y *= 0.3;
            r *= 0.3;
        }else if (currentGamepad1.right_trigger > 0.01) {
            x *= 0.65;
            y *= 0.65;
            r *= 0.65;
        }else if (currentGamepad1.left_trigger > 0.01) {
            x *= 0.65;
            y *= 0.65;
            r *= 0.65;
        }

        // attempting to save motor calls == faster frequency of command calls
        if ( !(previousX == x && previousY == y && previousR == r) ) {
            robot.drivetrainSubsystem.moveXYR(x, y, r);
        }

        previousX = x;
        previousY = y;
        previousR = r;
    }

    public void shooterControl(){
        if (currentGamepad1.a && !previousGamepad1.a) {robot.shooterSubsystem.togglePaddle();}
        robot.shooterSubsystem.setVelocity(currentGamepad1.right_trigger);
    }
}