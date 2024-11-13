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

    public static double FORWARD1 = -48;
    public static double BACK1 = -46;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        telemetry.update();

        PoseStorage.AUTO_SHIFT_YAW = 180.0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, specimenStart);

        telemetry.addLine("BEFORE3z");
        telemetry.update();

        TrajectoryActionBuilder moveToBar1 = drive.actionBuilder(new Pose2d(-8.65, -55, Math.toRadians(270)))
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(-8.65, -48), Math.toRadians(270))
                .setReversed(false);
        TrajectoryActionBuilder moveToBar2 = moveToBar1.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(-8.65, -46), Math.toRadians(270))
                .setReversed(false);
        TrajectoryActionBuilder moveToObservationZone = moveToBar2.endTrajectory().fresh()
                .waitSeconds(2)
                .splineTo(new Vector2d(-60, -60), Math.toRadians(0));

        telemetry.addLine("BEFORE2");
        telemetry.update();

        TrajectoryActionBuilder wait2 = drive.actionBuilder(specimenStart)
                .waitSeconds(2);
        TrajectoryActionBuilder wait1 = drive.actionBuilder(specimenStart)
                .waitSeconds(1);
        TrajectoryActionBuilder wait05 = drive.actionBuilder(specimenStart)
                .waitSeconds(0.5);

        telemetry.addLine("BEFORE");
        telemetry.update();

        Action moveToBar1b = moveToBar1.build();
//        Action moveToBar2b = moveToBar2.build();
//        Action moveToObservationZoneb = moveToObservationZone.build();

        telemetry.addLine("HERERGSGDS");
        telemetry.update();

        // actions that need to happen on init
//        Actions.runBlocking(clawSystem.closeClaw());

//        while (!isStopRequested() && opModeInInit()) {
//            telemetry.addData("TEAM COLOR", isBlue ? "BLUE" : "RED");
//            if (currentGamepad.x && !previousGamepad.x) {
//                isBlue = !isBlue;
//            }
//        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                    moveToBar1b
//                    new ParallelAction(
//                            armLift.updateArmSubsystem(),
//                            new SequentialAction(
//                                    moveToBar1.build(),
////                                    ,armLift.depositSpecimen(),
////                                    moveToBar2.build(),
////                                    armLift.pivotDown(),
////                                    wait05.build(),
////                                    clawSystem.openClaw(),
////                                    wait05.build(),
////                                    clawSystem.closeClaw(),
////                                    armLift.restArm(),
////                                    moveToObservationZone.build(),
//                                    armLift.stopUpdate()
//                            )
//                    )
        );



    }
}
