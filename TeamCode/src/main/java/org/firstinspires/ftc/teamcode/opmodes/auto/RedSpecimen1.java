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
@Autonomous (name="Red Specimen 1 (2+0) WIP", group="Testing")
public class RedSpecimen1 extends AutoBase {

    public static double FORWARD1 = -48;
    public static double BACK1 = -46;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        PoseStorage.AUTO_SHIFT_YAW = 180.0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, specimenStart);

        TrajectoryActionBuilder moveToBar1 = drive.actionBuilder(specimenStart)
                .waitSeconds(1)
                .splineTo(new Vector2d(FORWARD1, specimenStart.position.y), Math.toRadians(180));
        TrajectoryActionBuilder moveToBar2 = moveToBar1.fresh()
                .splineTo(new Vector2d(BACK1, specimenStart.position.y), Math.toRadians(180));
        TrajectoryActionBuilder moveToObservationZone = moveToBar2.fresh()
                .splineTo(new Vector2d(-48, -60), Math.toRadians(270))
                .turn(Math.toRadians(90));
        TrajectoryActionBuilder moveForward = moveToObservationZone.fresh()
                .lineToY(-50);
        TrajectoryActionBuilder moveBackToBar = moveForward.fresh()
                .turn(Math.toRadians(90));
            // FINISH THIS PATH LATER

        TrajectoryActionBuilder wait2 = drive.actionBuilder(specimenStart)
                .waitSeconds(2);
        TrajectoryActionBuilder wait1 = drive.actionBuilder(specimenStart)
                .waitSeconds(1);
        TrajectoryActionBuilder wait05 = drive.actionBuilder(specimenStart)
                .waitSeconds(0.5);

        // actions that need to happen on init
        Actions.runBlocking(clawSystem.closeClaw());

//        while (!isStopRequested() && opModeInInit()) {
//            telemetry.addData("TEAM COLOR", isBlue ? "BLUE" : "RED");
//            if (currentGamepad.x && !previousGamepad.x) {
//                isBlue = !isBlue;
//            }
//        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        armLift.updateArmSubsystem(),
                        new SequentialAction(
                                moveToBar1.build(),
                                armLift.depositSpecimen(),
                                moveToBar2.build(),
                                armLift.pivotDown(),
                                wait05.build(),
                                clawSystem.openClaw(),
                                wait05.build(),
                                clawSystem.closeClaw(),
                                armLift.restArm(),
                                moveToObservationZone.build(),
                                armLift.stopUpdate()
                        )
                )
        );

    }
}
