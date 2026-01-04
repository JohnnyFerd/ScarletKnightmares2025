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
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

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
    public static double turnAngle = -65;

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
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) {
            aprilTag.stop();
            return;
        }

        // ===== Shoot 3 balls at the start safely =====
        robot.outake.outakeOn();          // spin up flywheel
        sleep(300);                        // short spin-up delay

        for (int i = 0; i < 3; i++) {
            robot.spindexer.rotateByFraction(-1.0 / 3.0); // positive fraction

            while (!robot.spindexer.isIdle() && opModeIsActive()) {
                robot.spindexer.update();
                robot.outake.update();

            }
        }

        robot.outake.outakeOff();         // stop flywheel

        // ===== Move Back Before First Shot =====
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(drive.pose.position.x - 40, drive.pose.position.y))
                        .build()
        );

        // ===== Sequential Moves Relative to Current Pose =====
        Pose2d current = drive.pose;
        Action move2 = drive.actionBuilder(current)
                .turn(Math.toRadians(turnAngle))
                .strafeTo(new Vector2d(current.position.x + 25, current.position.y + 5))
                .build();
        Actions.runBlocking(move2);

        current = drive.pose;
        Action move3 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x + 8, current.position.y - 10))
                .build();
        Actions.runBlocking(move3);

        current = drive.pose;
        Action move4 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x - 20, current.position.y - 30))
                .build();
        Actions.runBlocking(move4);

        robot.intake.intakeOn(); // start intake for collecting balls

        current = drive.pose;
        Action move5 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x - 35, current.position.y))
                .build();
        Actions.runBlocking(move5);

        current = drive.pose;
        Action move6 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x + 35, current.position.y))
                .build();
        Actions.runBlocking(move6);

        current = drive.pose;
        Action move7 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x - move7x, current.position.y + move7y))
                .build();
        Actions.runBlocking(move7);

        current = drive.pose;
        Action move8 = drive.actionBuilder(current)
                .strafeTo(new Vector2d(current.position.x - move8x, current.position.y + move8y))
                .build();
        // Uncomment if you want to run this final move
        // Actions.runBlocking(move8);

        // Stop vision safely
        aprilTag.stop();
    }
}
