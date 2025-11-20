package org.firstinspires.ftc.teamcode.opmodes.auto;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@Autonomous(name = "PlanB_BLUE")
public class PlanB_BLUE extends AutoBase {

    private AprilTag aprilTag;
    public JVBoysSoccerRobot robot;
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry, true);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        aprilTag = robot.aprilTag;

        telemetry.addLine("Initializing camera...");
        telemetry.update();

        // Pre-start detection loop
        while (!isStarted() && !isStopRequested()) {
            aprilTag.update();
            aprilTag.addTelemetry();
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) {
            aprilTag.stop();
            return;
        }

        // Shooter update thread
        Thread shooterThread = new Thread(() -> {
            while (opModeIsActive()) {
                robot.shooterSubsystem.update();
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        shooterThread.start();

        // Small forward adjustment before turning
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(drive.pose.position.x - 2, drive.pose.position.y))
                        .turn(Math.toRadians(25))
                        .build()
        );

        // Update AprilTag after turn
        for (int i = 0; i < 20; i++) aprilTag.update();
        telemetry.addData("Detected goal", aprilTag.goalLabel);
        telemetry.update();

        // Spin up shooter at slightly lower velocity
        telemetry.addLine("Spinning up shooter...");
        telemetry.update();
        robot.shooterSubsystem.setVelocity(Shooter.FarShotVelo);
        sleep(2000);

        // Fire sequence
        robot.shooterSubsystem.paddleUp();
        sleep(2000);
        robot.shooterSubsystem.paddleDown();

        sleep(2000);
        robot.shooterSubsystem.paddleUp();
        sleep(2000);
        robot.shooterSubsystem.paddleDown();

        sleep(2000);
        robot.shooterSubsystem.paddleUpLast();
        sleep(2000);
        robot.shooterSubsystem.paddleDown();

        robot.shooterSubsystem.setVelocity(0);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(drive.pose.position.x - 15, drive.pose.position.y-10))
                        .turn(Math.toRadians(25))
                        .build()
        );
        // Stop vision and shooter safely
        robot.shooterSubsystem.update();
        robot.shooterSubsystem.stop();
        shooterThread.interrupt();
        aprilTag.stop();
    }
}
