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
@Autonomous(name = "PlanA_Blue")
public class PlanA_BLUE extends AutoBase {

    private AprilTag aprilTag;
    public JVBoysSoccerRobot robot;
    private MecanumDrive drive;
    public static double move7x = 20;
    public static double move7y = -15;
    public static double move8x = -5;
    public static double move8y = -10;
    public static double turnAngle = 65;

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

        // Move Back Before First Shot
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(drive.pose.position.x + 40, drive.pose.position.y))
                        .build()
        );
        // Spin up shooter at slightly lower velocity
        robot.shooterSubsystem.setVelocity(Shooter.CloseShotVelo+30);
        sleep(1000);

        // Fire sequence
        robot.shooterSubsystem.paddleUp();
        sleep(750);
        robot.shooterSubsystem.paddleDown();
        sleep(900);
        robot.shooterSubsystem.paddleUp();
        sleep(900);
        robot.shooterSubsystem.paddleDown();

        sleep(1750);
        robot.shooterSubsystem.paddleUp();
        sleep(1000);
        robot.shooterSubsystem.paddleDown();

        robot.shooterSubsystem.setVelocity(0);

        // Sequential moves relative to current pose

        Pose2d current = drive.pose;
        Action move2 = drive.actionBuilder(current)
                .turn(Math.toRadians(turnAngle))
                .strafeTo(new Vector2d(current.position.x-25, current.position.y+5))
                .build();
        Actions.runBlocking(move2);

        current = drive.pose;
        Action move3 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x-8, current.position.y-10))
                .build();
        Actions.runBlocking(move3);

        current = drive.pose;
        Action move4 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x+20, current.position.y-30))
                .build();
        Actions.runBlocking(move4);

        current = drive.pose;
        Action move5 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x+35, current.position.y))
                .build();
        Actions.runBlocking(move5);
        current = drive.pose;
        Action move6 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x-35, current.position.y))
                .build();
        Actions.runBlocking(move6);
        current = drive.pose;
        Action move7 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x+move7x, current.position.y+move7y))
                .build();
        Actions.runBlocking(move7);

        current=drive.pose;
        Action move8 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x+move8x, current.position.y+move8y))
                .build();
        //Actions.runBlocking(move8);

        // Stop vision and shooter safely
        robot.shooterSubsystem.update();
        robot.shooterSubsystem.stop();
        shooterThread.interrupt();
        aprilTag.stop();
    }
}
