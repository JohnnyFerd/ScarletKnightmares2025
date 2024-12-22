package org.firstinspires.ftc.teamcode.opmodes.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous (name="Red Specimen", group="Testing")
public class RedSpecimen extends AutoBase {

    private boolean choicePicked = false;
    private int pathNumber = 0;
    private MecanumDrive drive;

    // 1 specimen
    private Action moveToBar11, moveToBar21, moveToObservationZone1;
    private Action moveToBar12, moveToBar22, moveToObservationZone2, getSpecimen2, moveBackToBar12, moveBackToBar22, moveBackToObservationZone2;
    private Action groundSample1, groundSample2;

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
                twoSpecimenPathsAlternate();
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
                        new ParallelAction(
                                armLift.updateArmSubsystem(),
                                new SequentialAction(
                                        moveToBar11,
                                        armLift.depositSpecimen(),
                                        armLift.extendSlide(),
//                                        armLift.pivotDown(),
                                        new SleepAction(0.5),
                                        clawSystem.openClaw(),
                                        new SleepAction(0.5),
                                        moveToBar21,
                                        clawSystem.closeClaw(),
                                        armLift.deExtendSlide(),
                                        armLift.restArm(),
                                        moveToObservationZone1,
                                        armLift.stopUpdate()
                                )
                        )
                );
                break;
            case 2:
                Actions.runBlocking(
                        new ParallelAction(
                                armLift.updateArmSubsystem(),
                                new SequentialAction(
                                        moveToBar11,
                                        armLift.depositSpecimen(),
                                        armLift.extendSlide(),
//                                        armLift.pivotDown(),
                                        new SleepAction(0.5),
                                        clawSystem.openClaw(),
                                        new SleepAction(0.5),
                                        moveToBar21,
                                        clawSystem.closeClaw(),
                                        armLift.deExtendSlide(),
                                        moveToObservationZone2,
                                        armLift.intakeSpecimen(),
                                        clawSystem.openClaw(),
                                        new SleepAction(1),
                                        getSpecimen2,
                                        new SleepAction(0.5),
                                        clawSystem.closeClaw(),
                                        new SleepAction(0.5),
                                        armLift.depositSpecimen(),
                                        moveBackToBar12,
                                        armLift.extendSlide(),
//                                        armLift.pivotDown(),
                                        new SleepAction(0.75),
                                        clawSystem.openClaw(),
                                        new SleepAction(0.5),
                                        moveBackToBar22,
                                        clawSystem.closeClaw(),
                                        armLift.deExtendSlide(),
                                        armLift.restArm(),
                                        moveBackToObservationZone2,
                                        armLift.stopUpdate()
                                )
                        )
                );
                break;
            case 3:
//                Actions.runBlocking(
//                        new ParallelAction(
//                                armLift.updateArmSubsystem(),
//                                new SequentialAction(
//                                        moveToBar12,
//                                        armLift.depositSpecimen(),
//                                        new SleepAction(0.75),
//                                        armLift.pivotDown(),
//                                        new SleepAction(0.25),
//                                        clawSystem.openClaw(),
//                                        moveToBar22,
//                                        clawSystem.closeClaw(),
//                                        moveToObservationZone2,
//                                        clawSystem.openClaw(),
//                                        armLift.intakeSpecimenGround(),
//                                        new SleepAction(0.5),
//                                        clawSystem.closeClaw(),
//                                        new SleepAction(0.25),
//                                        armLift.depositSpecimen(),
//                                        moveBackToBar12,
//                                        armLift.pivotDown(),
//                                        new SleepAction(0.5),
//                                        clawSystem.openClaw(),
//                                        new SleepAction(0.25),
//                                        clawSystem.closeClaw(),
//                                        armLift.restArm(),
//                                        armLift.stopUpdate(),
//                                        groundSample1
//                                )
//                        )
//                );
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
        TrajectoryActionBuilder moveToBar1B = drive.actionBuilder(specimenStart)
                .lineToY(-45.5);
        TrajectoryActionBuilder moveToBar2B = moveToBar1B.endTrajectory().fresh()
                .lineToY(-47);
        TrajectoryActionBuilder moveToObservationZoneB = moveToBar2B.endTrajectory().fresh()
                .turn(Math.toRadians(90))
                .splineTo(new Vector2d(48, -43), Math.toRadians(0))
                .waitSeconds(0.5)
                .turn(Math.toRadians(90));
        TrajectoryActionBuilder getSpecimenB = moveToObservationZoneB.endTrajectory().fresh()
                .lineToY(-48);
        TrajectoryActionBuilder moveBackToBar1B = getSpecimenB.endTrajectory().fresh()
                .turn(Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(4, -48), Math.toRadians(90))
                .lineToY(-45);
        TrajectoryActionBuilder moveBackToBar2B = moveBackToBar1B.endTrajectory().fresh()
                .setReversed(false)
                .lineToY(-48);
        TrajectoryActionBuilder moveBackToObservationZoneB = moveBackToBar2B.endTrajectory().fresh()
                .splineTo(new Vector2d(60, -60), Math.toRadians(0));

        moveToBar12 = moveToBar1B.build();
        moveToBar22 = moveToBar2B.build();
        moveToObservationZone2 = moveToObservationZoneB.build();
        getSpecimen2 = getSpecimenB.build();
        moveBackToBar12 = moveBackToBar1B.build();
        moveBackToBar22 = moveBackToBar2B.build();
        moveBackToObservationZone2 = moveBackToObservationZoneB.build();

    }

    public void twoSpecimenPathsAlternate() {
        TrajectoryActionBuilder moveToBar1B = drive.actionBuilder(specimenStart)
                .lineToY(-45.5);
        TrajectoryActionBuilder moveToBar2B = moveToBar1B.endTrajectory().fresh()
                .lineToY(-46);
        TrajectoryActionBuilder moveToObservationZoneB = moveToBar2B.endTrajectory().fresh()
                .turn(Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(26, -60), Math.toRadians(0), new TranslationalVelConstraint(40));
//        TrajectoryActionBuilder getSpecimenB = moveToObservationZoneB.endTrajectory().fresh()
//                .lineToX(38);
        TrajectoryActionBuilder moveBackToBar1B = moveToObservationZoneB.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(2, -50), Math.toRadians(180))
                .turn(Math.toRadians(90))
                .lineToY(-48);
//        TrajectoryActionBuilder moveBackToBar2B = moveBackToBar1B.endTrajectory().fresh()
//                .lineToY(-50);
        TrajectoryActionBuilder groundSample1B = moveBackToBar1B.endTrajectory().fresh()
                .turn(Math.toRadians(90))
                .lineToX(30)
                .splineTo(new Vector2d(36, -12), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(48, -48), Math.toRadians(270))
                .lineToY(-60);
//        TrajectoryActionBuilder moveBackToObservationZoneB = moveBackToBar2B.endTrajectory().fresh()
//                .splineTo(new Vector2d(60, -60), Math.toRadians(0));

        moveToBar12 = moveToBar1B.build();
        moveToBar22 = moveToBar2B.build();
        moveToObservationZone2 = moveToObservationZoneB.build();
//        getSpecimen2 = getSpecimenB.build();
        moveBackToBar12 = moveBackToBar1B.build();
//        moveBackToBar22 = moveBackToBar2B.build();
        groundSample1 = groundSample1B.build();
//        moveBackToObservationZone2 = moveBackToObservationZoneB.build();
    }
}
