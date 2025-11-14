package org.firstinspires.ftc.teamcode.opmodes.auto;

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
@Autonomous(name = "PlanA_RED")
public class PlanA_RED extends AutoBase {

    private AprilTag aprilTag;
    public JVBoysSoccerRobot robot;
    private MecanumDrive drive;

    // Red uses same configurable offsets as Blue, but Y is mirrored
    public static double move7x = 20;
    public static double move7y = +15;   // flipped from -15

    public static double move8x = -5;
    public static double move8y = +10;   // flipped from -10

    public static double move9x = 60;
    public static double move9y = +25;   // flipped from -25

    public static double turnAngle = -60; // flipped from +60

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry, true);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        aprilTag = robot.aprilTag;

        telemetry.addLine("Initializing camera...");
        telemetry.update();

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

        Thread shooterThread = new Thread(() -> {
            while (opModeIsActive()) {
                robot.shooterSubsystem.update();
                try { Thread.sleep(10); }
                catch (InterruptedException e) { break; }
            }
        });
        shooterThread.start();

        // Move Back Before First Shot (Y stays same, angle flipped)
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(drive.pose.position.x + 35, drive.pose.position.y))
                        .turn(Math.toRadians(-5))   // flipped from +5
                        .build()
        );

        robot.shooterSubsystem.setVelocity(Shooter.CloseShotVelo);
        sleep(1000);

        // Fire sequence (unchanged)
        robot.shooterSubsystem.paddleUp();
        sleep(750);
        robot.shooterSubsystem.paddleDown();
        sleep(900);
        robot.shooterSubsystem.paddleUp();
        sleep(900);
        robot.shooterSubsystem.paddleDown();

        sleep(1250);
        robot.shooterSubsystem.paddleUp();
        sleep(1000);
        robot.shooterSubsystem.paddleDown();
        robot.shooterSubsystem.setVelocity(0);

        Pose2d current = drive.pose;

        // Move 2 (turn flipped, Y flipped)
        Action move2 = drive.actionBuilder(current)
                .turn(Math.toRadians(turnAngle))   // -60 now
                .strafeTo(new Vector2d(current.position.x - 25, current.position.y - 5))
                .build();
        Actions.runBlocking(move2);

        current = drive.pose;

        // Move 3 (Y flipped from -10 → +10)
        Action move3 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x - 8, current.position.y + 10))
                .build();
        Actions.runBlocking(move3);

        current = drive.pose;

        // Move 4 (Y flipped from -30 → +30)
        Action move4 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x + 20, current.position.y + 30))
                .build();
        Actions.runBlocking(move4);

        current = drive.pose;

        // Move 5 (Y stays same since it's 0)
        Action move5 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x + 35, current.position.y))
                .build();
        Actions.runBlocking(move5);

        current = drive.pose;

        // Move 6 (same pattern as Blue)
        Action move6 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x - 35, current.position.y))
                .build();
        Actions.runBlocking(move6);

        current = drive.pose;

        // Move 7 (Y flipped)
        Action move7 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x + move7x, current.position.y + move7y))
                .build();
        Actions.runBlocking(move7);

        current = drive.pose;

        // Move 8 (Y flipped)
        Action move8 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x + move8x, current.position.y + move8y))
                .build();
        Actions.runBlocking(move8);


        robot.shooterSubsystem.update();
        robot.shooterSubsystem.stop();
        shooterThread.interrupt();
        aprilTag.stop();
    }
}
