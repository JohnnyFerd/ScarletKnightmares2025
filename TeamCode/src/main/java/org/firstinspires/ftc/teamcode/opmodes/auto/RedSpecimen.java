package org.firstinspires.ftc.teamcode.opmodes.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
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
    private MecanumDrive drive;

    // 1 specimen
    private Action moveToBar11, moveToBar21, moveToObservationZone1;
    private Action moveToBar12, moveToBar22, moveToObservationZone2, getSpecimen2, moveBackToBar12, moveBackToBar22, moveBackToObservationZone2;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        PoseStorage.AUTO_SHIFT_YAW = 180.0;
        drive = new MecanumDrive(hardwareMap, specimenStart);

        while (!choicePicked) {
            telemetry.addLine("PICK NUMBER OF SPECIMEN");
            telemetry.addLine("A = 1, B = 2, X = 3, Y = 4");
            telemetry.update();
            if (currentGamepad.a && !previousGamepad.a) {
                pathNumber = 1;
                choicePicked = true;
            }
        }

        telemetry.addData("NUMBER OF SPECIMEN CHOSEN: ", pathNumber);
        telemetry.update();

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


        // actions that need to happen on init
        Actions.runBlocking(clawSystem.closeClaw());

        waitForStart();

        if (isStopRequested()) return;

        switch (pathNumber) {
            case 1:
                Actions.runBlocking(
                    new SequentialAction(
                            moveToBar11,
                            moveToBar21,
                            moveToObservationZone1
                    )
//                    new ParallelAction(
//                            armLift.updateArmSubsystem(),
//                            new SequentialAction(
//                                    moveToBar1.build(),
//                                    armLift.depositSpecimen(),
//                                    moveToBar2.build(),
//                                    armLift.pivotDown(),
//                                    new SleepAction(0.5),
//                                    clawSystem.openClaw(),
//                                    new SleepAction(0.5),
//                                    clawSystem.closeClaw(),
//                                    armLift.restArm(),
//                                    moveToObservationZone.build(),
//                                    armLift.stopUpdate()
//                            )
//                    )
                );
                break;
            case 2:
                break;
        }
    }

    public void oneSpecimenPaths() {
        TrajectoryActionBuilder moveToBar1B = drive.actionBuilder(specimenStart)
                .waitSeconds(1)
                .lineToY(-53);
        TrajectoryActionBuilder moveToBar2B = moveToBar1B.endTrajectory().fresh()
                .waitSeconds(1)
                .lineToY(-55);
        TrajectoryActionBuilder moveToObservationZoneB = moveToBar2B.endTrajectory().fresh()
                .waitSeconds(1)
                .splineTo(new Vector2d(60, -60), Math.toRadians(0));
//                .setReversed(false);

        moveToBar11 = moveToBar1B.build();
        moveToBar21 = moveToBar2B.build();
        moveToObservationZone1 = moveToObservationZoneB.build();
    }

    public void twoSpecimenPaths() {
        TrajectoryActionBuilder moveToBar1B = drive.actionBuilder(specimenStart)
                .waitSeconds(1)
                .lineToY(-50);
        TrajectoryActionBuilder moveToBar2B = moveToBar1B.endTrajectory().fresh()
                .waitSeconds(1)
                .lineToY(-52);
        TrajectoryActionBuilder moveToObservationZoneB = moveToBar2B.endTrajectory().fresh()
                .waitSeconds(1)
                .splineTo(new Vector2d(60, -56), Math.toRadians(0));
//                .setReversed(false);
        TrajectoryActionBuilder getSpecimenB = moveToObservationZoneB.endTrajectory().fresh()
                .setReversed(false)
                .turn(Math.toRadians(90))
                .lineToY(-57);
        TrajectoryActionBuilder moveBackToBar1B = getSpecimenB.endTrajectory().fresh()
                .turn(Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(2, -50), Math.toRadians(90));
        TrajectoryActionBuilder moveBackToBar2B = moveBackToBar1B.endTrajectory().fresh()
                .setReversed(false)
                .lineToY(-52);
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
}
