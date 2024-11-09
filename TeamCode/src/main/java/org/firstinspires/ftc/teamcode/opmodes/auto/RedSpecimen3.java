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
@Autonomous (name="Red Specimen 3 (0+0)", group="Testing")
public class RedSpecimen3 extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        Pose2d initialPose = new Pose2d(-17, -60, Math.toRadians(270));
        PoseStorage.AUTO_SHIFT_YAW = 90;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder moveToObservationZone = drive.actionBuilder(initialPose)
                .lineToX(-55);

        TrajectoryActionBuilder wait2 = drive.actionBuilder(initialPose)
                .waitSeconds(2);
        TrajectoryActionBuilder wait1 = drive.actionBuilder(initialPose)
                .waitSeconds(1);
        TrajectoryActionBuilder wait05 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);

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
                        moveToObservationZone.build()
                )
        );

    }
}
