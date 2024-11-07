package org.firstinspires.ftc.teamcode.opmodes.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous (name="Red Specimen 2 (1+0)", group="Testing")
public class RedSpecimen2 extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        Pose2d initialPose = new Pose2d(8.65, -55, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .lineToY(-24.5);
        TrajectoryActionBuilder driveBackwards = traj1.fresh()
                .lineToY(-26);
        TrajectoryActionBuilder traj4 = driveBackwards.fresh()
                .setReversed(true)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(60, -60), Math.toRadians(0));

        TrajectoryActionBuilder wait2 = drive.actionBuilder(initialPose)
                .waitSeconds(2);

        // actions that need to happen on init
        Actions.runBlocking(clawSystem.closeClaw());

        while (!isStopRequested() && opModeInInit()) {
            telemetry.addData("TEAM COLOR", isBlue ? "BLUE" : "RED");
            if (currentGamepad.x && !previousGamepad.x) {
                isBlue = !isBlue;
            }
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        traj1.build(),
                        new ParallelAction(
                                armLift.depositSpecimen(),
                                new SequentialAction(
                                        wait2.build(),
                                        driveBackwards.build(),
                                        clawSystem.openClaw(),
                                        armLift.restArm()
                                )
                        ),
                        traj4.build()
                )
        );

    }
}
