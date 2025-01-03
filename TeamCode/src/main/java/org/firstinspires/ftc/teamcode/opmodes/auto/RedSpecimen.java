package org.firstinspires.ftc.teamcode.opmodes.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous (name="Red Specimen", group="Testing")
public class RedSpecimen extends AutoBase {

    private boolean choicePicked = false;
    private int pathNumber = 0;
    private double timeDelay = 0;
    private MecanumDrive drive;

    private Action moveToBar11, moveToBar21, moveToObservationZone1;
    private Action depositFirstSpecimen1, depositFirstSpecimen2, pickUpSecondSpecimen1, pickUpSecondSpecimen2, depositSecondSpecimen1, depositSecondSpecimen2, moveBackToObservationZone2;
    private Action moveToFirstSample, moveToSecondSample;
    private Action pickUpThirdSpecimen1, pickUpThirdSpecimen2;
    private Action depositThirdSpecimen1, depositThirdSpecimen2;
    private Action pickUpFourthSpecimen1, pickUpFourthSpecimen2;
    private Action depositFourthSpecimen1, depositFourthSpecimen2;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        PoseStorage.AUTO_SHIFT_YAW = 0;
        drive = new MecanumDrive(hardwareMap, specimenStart);

        while (!choicePicked) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            telemetry.addLine("PICK NUMBER OF SPECIMEN");
            telemetry.addLine("Press A to increase path counter");
            telemetry.addLine("Press B to decrease path counter");
            telemetry.addLine("Press DPAD UP and DPAD DOWN to add starting delay");
            telemetry.addLine("Press X when ready");
            telemetry.update();
            if (currentGamepad.a && !previousGamepad.a) {
                pathNumber++;
            }
            if (currentGamepad.b && !previousGamepad.b) {
                pathNumber--;
            }
            if (currentGamepad.x && !previousGamepad.x) {
                choicePicked = true;
            }
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                timeDelay += 0.5;
            }
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                timeDelay -= 0.5;
                if (timeDelay < 0) {
                    timeDelay = 0;
                }
            }
            telemetry.addData("NUMBER OF SPECIMEN CHOSEN: ", pathNumber);
            telemetry.addData("Time Delay: ", timeDelay);
            if (isStopRequested()) return;
        }

        switch (pathNumber) {
            case 1:
                oneSpecimenPaths();
                break;
            case 2:
                twoSpecimenPaths();
                break;
            case 3:
                break;
            case 4:
                break;
        }

        telemetry.addData("NUMBER OF SPECIMEN CHOSEN: ", pathNumber);
        telemetry.update();

        // actions that need to happen on init
        Actions.runBlocking(clawSystem.closeClaw());

        waitForStart();

        if (isStopRequested()) return;

        switch (pathNumber) {
            case 1:
                Actions.runBlocking(
                        new SequentialAction(
                                new SleepAction(timeDelay),
                                new ParallelAction(
                                        armLift.updateArmSubsystem(),
                                        new SequentialAction(
                                                moveToBar11,
//                                                armLift.depositSpecimen(),
//                                                armLift.extendSlide(),
        //                                        armLift.pivotDown(),
                                                new SleepAction(0.5),
//                                                clawSystem.openClaw(),
                                                new SleepAction(0.5),
                                                moveToBar21,
//                                                clawSystem.closeClaw(),
//                                                armLift.deExtendSlide(),
//                                                armLift.restArm(),
                                                moveToObservationZone1,
                                                armLift.stopUpdate()
                                        )
                                )
                        )
                );
                break;
            case 2:
                Actions.runBlocking(
                        new SequentialAction(
                                new SleepAction(timeDelay),
                                new ParallelAction(
                                        armLift.updateArmSubsystem(),
                                        new SequentialAction(
                                                depositFirstSpecimen1,
//                                                armLift.depositSpecimen(),
//                                                armLift.extendSlide(),
        //                                        armLift.pivotDown(),
                                                new SleepAction(0.5),
//                                                clawSystem.openClaw(),
                                                new SleepAction(0.5),
                                                depositFirstSpecimen2,
//                                                clawSystem.closeClaw(),
//                                                armLift.deExtendSlide(),
                                                pickUpSecondSpecimen1,
//                                                armLift.intakeSpecimen(),
//                                                clawSystem.openClaw(),
                                                new SleepAction(1),
                                                pickUpSecondSpecimen2,
                                                new SleepAction(0.5),
//                                                clawSystem.closeClaw(),
                                                new SleepAction(0.5),
//                                                armLift.depositSpecimen(),
                                                depositSecondSpecimen1,
//                                                armLift.extendSlide(),
        //                                        armLift.pivotDown(),
                                                new SleepAction(0.5),
//                                                clawSystem.openClaw(),
                                                new SleepAction(0.5),
                                                depositSecondSpecimen2,
//                                                clawSystem.closeClaw(),
//                                                armLift.deExtendSlide(),
//                                                armLift.restArm(),
//                                                moveToFirstSample,
//                                                moveToSecondSample,
//
//                                                pickUpThirdSpecimen1,
////                                                armLift.intakeSpecimen(),
////                                                clawSystem.openClaw(),
//                                                new SleepAction(1),
//                                                pickUpThirdSpecimen2,
//                                                new SleepAction(0.5),
////                                                clawSystem.closeClaw(),
//                                                new SleepAction(0.5),
////                                                armLift.depositSpecimen(),
//                                                depositThirdSpecimen1,
////                                                armLift.extendSlide(),
//        //                                        armLift.pivotDown(),
//                                                new SleepAction(0.5),
////                                                clawSystem.openClaw(),
//                                                new SleepAction(0.5),
//                                                depositThirdSpecimen2,
////                                                clawSystem.closeClaw(),
////                                                armLift.deExtendSlide(),
//
//                                                pickUpFourthSpecimen1,
////                                                armLift.intakeSpecimen(),
////                                                clawSystem.openClaw(),
//                                                new SleepAction(1),
//                                                pickUpFourthSpecimen2,
//                                                new SleepAction(0.5),
////                                                clawSystem.closeClaw(),
//                                                new SleepAction(0.5),
////                                                armLift.depositSpecimen(),
//                                                depositFourthSpecimen1,
////                                                armLift.extendSlide(),
//        //                                        armLift.pivotDown(),
//                                                new SleepAction(0.5),
////                                                clawSystem.openClaw(),
//                                                new SleepAction(0.5),
//                                                depositFourthSpecimen2,
////                                                clawSystem.closeClaw(),
////                                                armLift.deExtendSlide(),
//
////                                                armLift.restArm(),
//                                                moveBackToObservationZone2,
                                                armLift.stopUpdate()
                                        )
                                )
                        )
                );
                break;
            case 3:
                break;
            case 4:
                break;
        }
    }

    public void oneSpecimenPaths() {
        TrajectoryActionBuilder moveToBar1B = drive.actionBuilder(specimenStart)
                .lineToY(-45.5);
        TrajectoryActionBuilder moveToBar2B = moveToBar1B.endTrajectory().fresh()
                .lineToY(-47);
        TrajectoryActionBuilder moveToObservationZoneB = moveToBar2B.endTrajectory().fresh()
                .splineTo(new Vector2d(46, -62), Math.toRadians(0));
//                .setReversed(false);

        moveToBar11 = moveToBar1B.build();
        moveToBar21 = moveToBar2B.build();
        moveToObservationZone1 = moveToObservationZoneB.build();
    }

    public void twoSpecimenPaths() {
        TrajectoryActionBuilder depositFirstSpecimen1B = drive.actionBuilder(specimenStart)
                .lineToY(-39);
        TrajectoryActionBuilder depositFirstSpecimen2B = depositFirstSpecimen1B.endTrajectory().fresh()
                .lineToY(-41);
        TrajectoryActionBuilder pickUpSecondSpecimen1B = depositFirstSpecimen2B.endTrajectory().fresh()
                .setTangent(Math.toRadians(0)) // beginning tangent
                .splineToLinearHeading(new Pose2d(36, -52, Math.toRadians(270)), Math.toRadians(0)) // end tangent
                .waitSeconds(0.5);
        TrajectoryActionBuilder pickUpSecondSpecimen2B = pickUpSecondSpecimen1B.endTrajectory().fresh()
                .strafeTo(new Vector2d(36, -55));
        TrajectoryActionBuilder depositSecondSpecimen1B = pickUpSecondSpecimen2B.endTrajectory().fresh()
                .setTangent(Math.toRadians(180)) // beginning tangent
                .splineToLinearHeading(new Pose2d(6, -39, Math.toRadians(90)), Math.toRadians(180)); // end tangent
        TrajectoryActionBuilder depositSecondSpecimen2B = depositSecondSpecimen1B.endTrajectory().fresh()
                .strafeTo(new Vector2d(6, -41));
        TrajectoryActionBuilder moveToFirstSampleB = depositSecondSpecimen2B.endTrajectory().fresh()
                .setTangent(Math.toRadians(270)) // beginning tangent
                .splineToLinearHeading(new Pose2d(36, -48, Math.toRadians(0)), Math.toRadians(90)) // end tangent;
                .lineToY(-12)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45, -48), Math.toRadians(270));
        TrajectoryActionBuilder moveToSecondSampleB = moveToFirstSampleB.endTrajectory().fresh()
                .lineToY( -12)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(54, -48), Math.toRadians(270));
        TrajectoryActionBuilder pickUpThirdSpecimen1B = moveToSecondSampleB.endTrajectory().fresh()
                .setTangent(Math.toRadians(180)) // beginning tangent
                .splineToLinearHeading(new Pose2d(36, -52, Math.toRadians(270)), Math.toRadians(180)); // end tangent
        TrajectoryActionBuilder pickUpThirdSpecimen2B = pickUpThirdSpecimen1B.endTrajectory().fresh()
                .strafeTo(new Vector2d(36, -55));
        TrajectoryActionBuilder depositThirdSpecimen1B = pickUpThirdSpecimen2B.endTrajectory().fresh()
                .setTangent(Math.toRadians(180)) // beginning tangent
                .splineToLinearHeading(new Pose2d(10, -39, Math.toRadians(90)), Math.toRadians(180)); // end tangent;
        TrajectoryActionBuilder depositThirdSpecimen2B = depositThirdSpecimen1B.endTrajectory().fresh()
                .strafeTo(new Vector2d(10, -41));
        TrajectoryActionBuilder pickUpFourthSpecimen1B = depositThirdSpecimen2B.endTrajectory().fresh()
                .setTangent(Math.toRadians(180)) // beginning tangent
                .splineToLinearHeading(new Pose2d(36, -52, Math.toRadians(270)), Math.toRadians(180)); // end tangent
        TrajectoryActionBuilder pickUpFourthSpecimen2B = pickUpFourthSpecimen1B.endTrajectory().fresh()
                .strafeTo(new Vector2d(36, -55));
        TrajectoryActionBuilder depositFourthSpecimen1B = pickUpFourthSpecimen2B.endTrajectory().fresh()
                .setTangent(Math.toRadians(180)) // beginning tangent
                .splineToLinearHeading(new Pose2d(4, -39, Math.toRadians(90)), Math.toRadians(180)); // end tangent;
        TrajectoryActionBuilder depositFourthSpecimen2B = depositFourthSpecimen1B.endTrajectory().fresh()
                .strafeTo(new Vector2d(4, -41));

        TrajectoryActionBuilder moveBackToObservationZoneB = depositFourthSpecimen2B.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(60, -60), Math.toRadians(0));

        depositFirstSpecimen1 = depositFirstSpecimen1B.build();
        depositFirstSpecimen2 = depositFirstSpecimen2B.build();
        pickUpSecondSpecimen1 = pickUpSecondSpecimen1B.build();
        pickUpSecondSpecimen2 = pickUpSecondSpecimen2B.build();
        depositSecondSpecimen1 = depositSecondSpecimen1B.build();
        depositSecondSpecimen2 = depositSecondSpecimen2B.build();
        moveToFirstSample = moveToFirstSampleB.build();
        moveToSecondSample = moveToSecondSampleB.build();
        pickUpThirdSpecimen1 = pickUpThirdSpecimen1B.build();
        pickUpThirdSpecimen2 = pickUpThirdSpecimen2B.build();
        depositThirdSpecimen1 = depositThirdSpecimen1B.build();
        depositThirdSpecimen2 = depositThirdSpecimen2B.build();
        pickUpFourthSpecimen1 = pickUpFourthSpecimen1B.build();
        pickUpFourthSpecimen2 = pickUpFourthSpecimen2B.build();
        depositFourthSpecimen1 = depositFourthSpecimen1B.build();
        depositFourthSpecimen2 = depositFourthSpecimen2B.build();

        moveBackToObservationZone2 = moveBackToObservationZoneB.build();
    }
}
