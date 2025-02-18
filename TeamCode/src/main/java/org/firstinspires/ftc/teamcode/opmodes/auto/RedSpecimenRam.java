package org.firstinspires.ftc.teamcode.opmodes.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

import java.util.Arrays;

@Config
@Autonomous (name="Red Specimen Ram", group="Testing")
public class RedSpecimenRam extends AutoBase {

    private boolean choicePicked = false;
    private int pathNumber = 1;
    private double timeDelay = 0;
    private MecanumDrive drive;

    private Action moveToBar11, moveToBar21, moveToObservationZone1;
    private Action depositFirstSpecimen1, depositFirstSpecimen2, pickUpSecondSpecimen1, pickUpSecondSpecimen2, depositSecondSpecimen1, depositSecondSpecimen2, moveBackToObservationZone2;
    private Action moveToFirstSample, moveToSecondSample;
    private Action moveToFirstSample2, moveToSecondSample2;
    private Action pickUpThirdSpecimen1, pickUpThirdSpecimen2;
    private Action depositThirdSpecimen1, depositThirdSpecimen2;
    private Action pickUpFourthSpecimen1, pickUpFourthSpecimen2;
    private Action depositFourthSpecimen1, depositFourthSpecimen2;

    private VelConstraint midVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(30.0),
            new AngularVelConstraint(Math.PI)
    ));
    private AccelConstraint midAccelConstraint = new ProfileAccelConstraint(-15, 30.0);

    private VelConstraint maxVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(50.0),
            new AngularVelConstraint(Math.PI)
    ));
    private AccelConstraint maxAccelConstraint = new ProfileAccelConstraint(-50.0, 50.0);



    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        Arm.DEFAULT_MAX_ACCELERATION = Arm.AUTO_MAX_ACCELERATION;
        Arm.DEFAULT_MAX_VELOCITY = Arm.AUTO_MAX_VELOCITY;
        Arm.DEFAULT_MAX_DECELERATION = Arm.AUTO_MAX_DECELERATION;

        PoseStorage.AUTO_SHIFT_YAW = 180;

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

        drive = new MecanumDrive(hardwareMap, specimenStart4);

        switch (pathNumber) {
            case 1:
                fourSpecimenPaths();
                break;
            case 2:
                fourSpecimenPathsOld();
                break;
        }

        telemetry.addData("NUMBER OF SPECIMEN CHOSEN: ", pathNumber);
        telemetry.update();

        // actions that need to happen on init
        Actions.runBlocking(
                new ParallelAction(
                        clawSystem.clawWrist0(),
                        clawSystem.closeClaw()
                )
        );

        waitForStart();

        if (isStopRequested()) return;

        switch (pathNumber) {
            case 1:
                Actions.runBlocking(
                        new SequentialAction(
                                clawSystem.clawWrist0(),
                                new SleepAction(timeDelay),
                                new ParallelAction(
                                        armLift.updateArmSubsystem(),
                                        new SequentialAction(
                                                new ParallelAction(
                                                        new SequentialAction(
                                                                new SleepAction(0.2),
                                                                depositFirstSpecimen1
                                                        ),
                                                        armLift.depositSpecimenFront()
                                                ),
                                                depositFirstSpecimen2,
                                                armLift.depositSpecimenFrontUp(),
                                                new SleepAction(0.15),
                                                clawSystem.openClaw(),
                                                new SleepAction(0.15),

                                                // push first sample
                                                new ParallelAction(
                                                        armLift.intakeSample(),
                                                        moveToFirstSample,
                                                        new SequentialAction(
                                                                new SleepAction(0.5),
                                                                clawSystem.clawWrist45()
                                                        )
                                                ),
                                                armLift.intakeSampleDown(),
                                                new SleepAction(0.2),
                                                clawSystem.closeClawTight(),
                                                new SleepAction(0.2),
                                                armLift.intakeSampleUp(),
                                                moveToFirstSample2,
                                                clawSystem.openClaw(),

                                                // push second sample
                                                moveToSecondSample,
                                                armLift.intakeSampleDown(),
                                                new SleepAction(0.2),
                                                clawSystem.closeClawTight(),
                                                new SleepAction(0.2),
                                                armLift.intakeSampleUp(),
                                                moveToSecondSample2,
                                                clawSystem.openClaw(),

                                                // pick up second specimen
                                                clawSystem.clawWrist0(),
                                                new ParallelAction(
                                                        new SequentialAction(
                                                                new SleepAction(0.25),
                                                                pickUpSecondSpecimen1
                                                        ),
                                                        armLift.intakeSpecimenPivotTimed()
                                                ),
                                                pickUpSecondSpecimen2,
                                                clawSystem.closeClawTight(),
                                                new SleepAction(0.15),

                                                // deposit second specimen
                                                new ParallelAction(
                                                        armLift.depositSpecimenRam(),
                                                        new SequentialAction(
                                                                new SleepAction(0.1),
                                                                clawSystem.clawWrist180(),
                                                                depositSecondSpecimen1
                                                        )
                                                ),
                                                depositSecondSpecimen2,
                                                new SleepAction(0.20),
                                                clawSystem.openClaw(),
                                                new SleepAction(0.15),

                                                // pick up third specimen
                                                new ParallelAction(
                                                        pickUpThirdSpecimen1,
                                                        new SequentialAction(
                                                                new SleepAction(0.5),
                                                                clawSystem.clawWrist0(),
                                                                armLift.intakeSpecimen()
                                                        )
                                                ),
                                                pickUpThirdSpecimen2,
                                                clawSystem.closeClawTight(),
                                                new SleepAction(0.15),

                                                // deposit third specimen
                                                new ParallelAction(
                                                        armLift.depositSpecimenRam(),
                                                        new SequentialAction(
                                                                new SleepAction(0.1),
                                                                clawSystem.clawWrist180(),
                                                                depositThirdSpecimen1
                                                        )
                                                ),
                                                depositThirdSpecimen2,
                                                new SleepAction(0.20),
                                                clawSystem.openClaw(),
                                                new SleepAction(0.15),

                                                new ParallelAction(
                                                        new SequentialAction(
                                                                new SleepAction(0.5),
                                                                new ParallelAction(
                                                                        armLift.restArm(),
                                                                        new SequentialAction(
                                                                                new SleepAction(0.25),
                                                                                clawSystem.clawWrist0(),
                                                                                clawSystem.closeClaw()
                                                                        )
                                                                )
                                                        ),
                                                        moveBackToObservationZone2
                                                ),
                                                armLift.stopUpdate()
                                        )
                                )
                        )
                );
                break;
            case 2:
                Actions.runBlocking(
                        new SequentialAction(
                                clawSystem.clawWrist0(),
                                new SleepAction(timeDelay),
                                new ParallelAction(
                                        armLift.updateArmSubsystem(),
                                        new SequentialAction(
                                                new ParallelAction(
                                                        new SequentialAction(
                                                                new SleepAction(0.2),
                                                                depositFirstSpecimen1
                                                        ),
                                                        armLift.depositSpecimenFront()
                                                ),
                                                depositFirstSpecimen2,
                                                armLift.depositSpecimenFrontUp(),
                                                new SleepAction(0.15),
                                                clawSystem.openClaw(),
                                                new SleepAction(0.15),

                                                // push first sample
                                                new ParallelAction(
                                                        armLift.intakeSample(),
                                                        moveToFirstSample,
                                                        new SequentialAction(
                                                                new SleepAction(0.5),
                                                                clawSystem.clawWrist45()
                                                        )
                                                ),
                                                armLift.intakeSampleDown(),
                                                new SleepAction(0.2),
                                                clawSystem.closeClaw(),
                                                new SleepAction(0.2),
                                                armLift.intakeSampleUp(),
                                                moveToFirstSample2,
                                                clawSystem.openClaw(),

                                                // push second sample
                                                moveToSecondSample,
                                                armLift.intakeSampleDown(),
                                                new SleepAction(0.2),
                                                clawSystem.closeClaw(),
                                                new SleepAction(0.2),
                                                armLift.intakeSampleUp(),
                                                moveToSecondSample2,
                                                clawSystem.openClaw(),

                                                // pick up second specimen
                                                clawSystem.clawWrist0(),
                                                new ParallelAction(
                                                        new SequentialAction(
                                                                new SleepAction(0.25),
                                                                pickUpSecondSpecimen1
                                                        ),
                                                        armLift.intakeSpecimenPivotTimed()
                                                ),
                                                pickUpSecondSpecimen2,
                                                clawSystem.closeClaw(),
                                                new SleepAction(0.15),

                                                // deposit second specimen
                                                new ParallelAction(
                                                        armLift.depositSpecimen(),
                                                        new SequentialAction(
                                                                new SleepAction(0.1),
                                                                depositSecondSpecimen1
                                                        )
                                                ),
//                                                depositSecondSpecimen2,
                                                armLift.depositSpecimenDown(),
                                                new SleepAction(0.20),
                                                clawSystem.openClaw(),
                                                new SleepAction(0.15),

                                                // pick up third specimen
                                                new ParallelAction(
                                                        pickUpThirdSpecimen1,
                                                        armLift.intakeSpecimen()
                                                ),
//                                                pickUpThirdSpecimen2,
                                                clawSystem.closeClaw(),
                                                new SleepAction(0.15),

                                                // deposit third specimen
                                                new ParallelAction(
                                                        armLift.depositSpecimen(),
                                                        new SequentialAction(
                                                                new SleepAction(0.1),
                                                                depositThirdSpecimen1
                                                        )
                                                ),
//                                                depositThirdSpecimen2,
                                                armLift.depositSpecimenDown(),
                                                new SleepAction(0.20),
                                                clawSystem.openClaw(),

                                                // pick up fourth specimen
                                                new ParallelAction(
                                                        pickUpFourthSpecimen1,
                                                        armLift.intakeSpecimen()
                                                ),
//                                                pickUpFourthSpecimen2,
                                                clawSystem.closeClaw(),
                                                new SleepAction(0.15),

                                                // deposit fourth specimen
                                                new ParallelAction(
                                                        armLift.depositSpecimen(),
                                                        new SequentialAction(
                                                                new SleepAction(0.1),
                                                                depositFourthSpecimen1
                                                        )
                                                ),
//                                                depositFourthSpecimen2,
                                                armLift.depositSpecimenDown(),
                                                new SleepAction(0.20),
                                                clawSystem.openClaw(),

                                                new SleepAction(0.15),
                                                new ParallelAction(
                                                        moveBackToObservationZone2,
                                                        new SequentialAction(
                                                                new SleepAction(0.5),
                                                                new ParallelAction(
                                                                        armLift.restArm(),
                                                                        new SequentialAction(
                                                                                new SleepAction(0.25),
                                                                                clawSystem.closeClaw()
                                                                        )
                                                                )
                                                        )
                                                ),

                                                armLift.stopUpdate()
                                        )
                                )
                        )
                );
                break;
        }
        Arm.DEFAULT_MAX_ACCELERATION = Arm.TELEOP_MAX_ACCELERATION;
        Arm.DEFAULT_MAX_VELOCITY = Arm.TELEOP_MAX_VELOCITY;
        Arm.DEFAULT_MAX_DECELERATION = Arm.TELEOP_MAX_DECELERATION;
    }

    public void fourSpecimenPaths() {
        TrajectoryActionBuilder depositFirstSpecimen1B = drive.actionBuilderFast(specimenStart4)
                .lineToY(-36.5, midVelConstraint, midAccelConstraint);
        TrajectoryActionBuilder depositFirstSpecimen2B = depositFirstSpecimen1B.endTrajectory().fresh()
                .lineToY(-36.5);
        TrajectoryActionBuilder pushFirstSampleB = drive.actionBuilder(new Pose2d(specimenStartX, -36.5, Math.toRadians(90)))
                .setReversed(true)
//                .setTangent(Math.toRadians(270))
//                .splineToSplineHeading(new Pose2d(32, -41, Math.toRadians(225)), Math.toRadians(45));
                .strafeToSplineHeading(new Vector2d(32, -42), Math.toRadians(225));
//                .strafeToConstantHeading(new Vector2d(49, -49));
        TrajectoryActionBuilder pushFirstSample2B = drive.actionBuilderFast(new Pose2d(32, -42, Math.toRadians(225)))
                .setReversed(true)
                .strafeToSplineHeading(new Vector2d(42, -46), Math.toRadians(135));
        TrajectoryActionBuilder pushSecondSampleB = pushFirstSample2B.endTrajectory().fresh()
                .setReversed(true)
                .strafeToSplineHeading(new Vector2d(42, -42), Math.toRadians(225));
        TrajectoryActionBuilder pushSecondSample2B = pushSecondSampleB.endTrajectory().fresh()
                .setReversed(true)
                .strafeToSplineHeading(new Vector2d(42, -46), Math.toRadians(135));
        TrajectoryActionBuilder intakeSecondSpecimenB = drive.actionBuilder(new Pose2d(42, -46, Math.toRadians(135)))
//                .setTangent(Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(36, -57), Math.toRadians(270));
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(42, -45), Math.toRadians(270));
//                .strafeToConstantHeading(new Vector2d(59, -57));
//        TrajectoryActionBuilder intakeSecondSpecimenB = pushSecondSample2B.endTrajectory().fresh()
////                .setTangent(Math.toRadians(180))
////                .splineToConstantHeading(new Vector2d(36, -57), Math.toRadians(270));
//                .setReversed(false)
//                .strafeToLinearHeading(new Vector2d(42, -45), Math.toRadians(270));
////                .strafeToConstantHeading(new Vector2d(59, -57));
        TrajectoryActionBuilder intakeSecondSpecimen2B = intakeSecondSpecimenB.endTrajectory().fresh()
                .strafeTo(new Vector2d(42, -54));
        TrajectoryActionBuilder depositSecondSpecimen1B = intakeSecondSpecimen2B.endTrajectory().fresh()
                .setReversed(false)
//                .setTangent(180)
//                .splineToSplineHeading(new Pose2d(2, -38, Math.toRadians(90)), Math.toRadians(90));
                .strafeToConstantHeading(new Vector2d(0, -55));
        TrajectoryActionBuilder depositSecondSpecimen2B = depositSecondSpecimen1B.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -41), null, maxAccelConstraint);
        TrajectoryActionBuilder pickUpThirdSpecimen1B = depositSecondSpecimen2B.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36, -50), Math.toRadians(270));
        TrajectoryActionBuilder pickUpThirdSpecimen2B = pickUpThirdSpecimen1B.endTrajectory().fresh()
                .strafeTo(new Vector2d(36, -55));
        TrajectoryActionBuilder depositThirdSpecimen1B = pickUpThirdSpecimen2B.endTrajectory().fresh()
//                .setTangent(180)
//                .splineToSplineHeading(new Pose2d(5, -38, Math.toRadians(90)), Math.toRadians(90));
                .strafeToConstantHeading(new Vector2d(-2, -55));
        TrajectoryActionBuilder depositThirdSpecimen2B = depositThirdSpecimen1B.endTrajectory().fresh()
                .strafeTo(new Vector2d(-2, -41), null, maxAccelConstraint);
        TrajectoryActionBuilder moveBackToObservationZoneB = depositThirdSpecimen2B.endTrajectory().fresh()
                .lineToY(-50)
                .strafeToLinearHeading(new Vector2d(60, -60), Math.toRadians(90));

        depositFirstSpecimen1 = depositFirstSpecimen1B.build();
        depositFirstSpecimen2 = depositFirstSpecimen2B.build();
        moveToFirstSample = pushFirstSampleB.build();
        moveToFirstSample2 = pushFirstSample2B.build();
        moveToSecondSample = pushSecondSampleB.build();
        moveToSecondSample2 = pushSecondSample2B.build();
        pickUpSecondSpecimen1 = intakeSecondSpecimenB.build();
        pickUpSecondSpecimen2 = intakeSecondSpecimen2B.build();
        depositSecondSpecimen1 = depositSecondSpecimen1B.build();
        depositSecondSpecimen2 = depositSecondSpecimen2B.build();
        pickUpThirdSpecimen1 = pickUpThirdSpecimen1B.build();
        pickUpThirdSpecimen2 = pickUpThirdSpecimen2B.build();
        depositThirdSpecimen1 = depositThirdSpecimen1B.build();
        depositThirdSpecimen2 = depositThirdSpecimen2B.build();
//        pickUpFourthSpecimen1 = pickUpFourthSpecimen1B.build();
//        pickUpFourthSpecimen2 = pickUpFourthSpecimen2B.build();
//        depositFourthSpecimen1 = depositFourthSpecimen1B.build();
//        depositFourthSpecimen2 = depositFourthSpecimen2B.build();

        moveBackToObservationZone2 = moveBackToObservationZoneB.build();

    }

    public void fourSpecimenPathsOld() {
        TrajectoryActionBuilder depositFirstSpecimen1B = drive.actionBuilderFast(specimenStart4)
                .lineToY(-36.5, midVelConstraint, midAccelConstraint);
        TrajectoryActionBuilder depositFirstSpecimen2B = depositFirstSpecimen1B.endTrajectory().fresh()
                .lineToY(-36.5);
        TrajectoryActionBuilder pushFirstSampleB = drive.actionBuilder(new Pose2d(specimenStartX, -36.5, Math.toRadians(90)))
                .setReversed(true)
//                .setTangent(Math.toRadians(270))
//                .splineToSplineHeading(new Pose2d(32, -41, Math.toRadians(225)), Math.toRadians(45));
                .strafeToSplineHeading(new Vector2d(32, -43), Math.toRadians(225));
//                .strafeToConstantHeading(new Vector2d(49, -49));
        TrajectoryActionBuilder pushFirstSample2B = drive.actionBuilderFast(new Pose2d(32, -42, Math.toRadians(225)))
                .setReversed(true)
                .strafeToSplineHeading(new Vector2d(42, -46), Math.toRadians(135));
        TrajectoryActionBuilder pushSecondSampleB = drive.actionBuilder(new Pose2d(42, -46, Math.toRadians(135)))
                .setReversed(true)
                .strafeToSplineHeading(new Vector2d(42, -41), Math.toRadians(225));
        TrajectoryActionBuilder pushSecondSample2B = drive.actionBuilderFast(new Pose2d(42, -41, Math.toRadians(225)))
                .setReversed(true)
                .strafeToSplineHeading(new Vector2d(42, -46), Math.toRadians(135));
        TrajectoryActionBuilder intakeSecondSpecimenB = drive.actionBuilder(new Pose2d(42, -46, Math.toRadians(135)))
//                .setTangent(Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(36, -57), Math.toRadians(270));
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(42, -45), Math.toRadians(270));
        TrajectoryActionBuilder intakeSecondSpecimen2B = intakeSecondSpecimenB.endTrajectory().fresh()
                .strafeTo(new Vector2d(42, -54));
        TrajectoryActionBuilder depositSecondSpecimen1B = intakeSecondSpecimen2B.endTrajectory().fresh()
//                .setReversed(false)
////                .setTangent(180)
////                .splineToSplineHeading(new Pose2d(2, -38, Math.toRadians(90)), Math.toRadians(90));
//                .strafeToConstantHeading(new Vector2d(2, -55))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(1.5, -46.5), Math.toRadians(90));
        TrajectoryActionBuilder depositSecondSpecimen2B = drive.actionBuilderFast(new Pose2d(1.5, -55, Math.toRadians(270)))
                .strafeTo(new Vector2d(1.5, -46.5));
        TrajectoryActionBuilder pickUpThirdSpecimen1B = drive.actionBuilder(new Pose2d(1.5, -46.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36, -55), Math.toRadians(270), midVelConstraint);
        TrajectoryActionBuilder pickUpThirdSpecimen2B = pickUpThirdSpecimen1B.endTrajectory().fresh()
                .strafeTo(new Vector2d(36, -55));
        TrajectoryActionBuilder depositThirdSpecimen1B = pickUpThirdSpecimen2B.endTrajectory().fresh()
//                .setTangent(180)
//                .splineToSplineHeading(new Pose2d(5, -38, Math.toRadians(90)), Math.toRadians(90));
//                .strafeToConstantHeading(new Vector2d(-2, -55));
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-2, -46.5), Math.toRadians(90));
        TrajectoryActionBuilder depositThirdSpecimen2B = drive.actionBuilderFast(new Pose2d(-2, -55, Math.toRadians(270)))
                .strafeTo(new Vector2d(-2, -46.5));

        TrajectoryActionBuilder pickUpFourthSpecimen1B = drive.actionBuilder(new Pose2d(-2, -46.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36, -55), Math.toRadians(270), midVelConstraint);
        TrajectoryActionBuilder pickUpFourthSpecimen2B = pickUpFourthSpecimen1B.endTrajectory().fresh()
                .strafeTo(new Vector2d(36, -55));
        TrajectoryActionBuilder depositFourthSpecimen1B = pickUpFourthSpecimen2B.endTrajectory().fresh()
//                .setTangent(180)
//                .splineToSplineHeading(new Pose2d(5, -38, Math.toRadians(90)), Math.toRadians(90));
//                .strafeToConstantHeading(new Vector2d(-6, -55));
                 .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-6, -46.5), Math.toRadians(90));
        TrajectoryActionBuilder depositFourthSpecimen2B = drive.actionBuilderFast(new Pose2d(-6, -55, Math.toRadians(270)))
                .strafeTo(new Vector2d(-6, -46.5));
        TrajectoryActionBuilder moveBackToObservationZoneB = drive.actionBuilder(new Pose2d(-6, -46.5, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(55, -55), Math.toRadians(90));

        depositFirstSpecimen1 = depositFirstSpecimen1B.build();
        depositFirstSpecimen2 = depositFirstSpecimen2B.build();
        moveToFirstSample = pushFirstSampleB.build();
        moveToFirstSample2 = pushFirstSample2B.build();
        moveToSecondSample = pushSecondSampleB.build();
        moveToSecondSample2 = pushSecondSample2B.build();
        pickUpSecondSpecimen1 = intakeSecondSpecimenB.build();
        pickUpSecondSpecimen2 = intakeSecondSpecimen2B.build();
        depositSecondSpecimen1 = depositSecondSpecimen1B.build();
        depositSecondSpecimen2 = depositSecondSpecimen2B.build();
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
