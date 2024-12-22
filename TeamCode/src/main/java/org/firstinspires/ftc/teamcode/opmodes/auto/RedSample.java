package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class RedSample extends AutoBase {

    private boolean choicePicked = false;
    private boolean preloadSample = true;
    private double timeDelay = 0;
    private MecanumDrive drive;

    private Action depositFirstSample, pickUpSecondSample, depositSecondSample, pickUpThirdSample, depositThirdSample, pickUpFourthSample, depositFourthSample;
    private Action park;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        PoseStorage.AUTO_SHIFT_YAW = 0;
        drive = new MecanumDrive(hardwareMap, specimenStart);

        while (!choicePicked) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            telemetry.addLine("PICK PRELOAD");
            telemetry.addLine("Press A to switch");
            telemetry.addLine("Press DPAD UP and DPAD DOWN to add starting delay");
            telemetry.addLine("Press X when ready");
            telemetry.update();
            if (currentGamepad.a && !previousGamepad.a) {
                preloadSample = !preloadSample;
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
            telemetry.addData("PRELOAD: ", preloadSample ? "Sample" : "Specimen");
            telemetry.addData("Time Delay: ", timeDelay);
            if (isStopRequested()) return;
        }

        if (preloadSample) {
            samplePaths();
        }else {
            specimenPaths();
        }

        // actions that need to happen on init
        Actions.runBlocking(clawSystem.closeClaw());

        waitForStart();
        telemetry.clear();
        if (isStopRequested()) return;

        if (preloadSample) {
            Actions.runBlocking(
                    new SleepAction(timeDelay),
                    new ParallelAction(
                            armLift.updateArmSubsystem(),
                            new SequentialAction(
                                    depositFirstSample,
                                    armLift.depositSample(),
                                    armLift.extendSlide(),
                                    new SleepAction(0.5),
                                    clawSystem.openClaw(),
                                    new SleepAction(0.5),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    new SleepAction(0.25),
                                                    armLift.deExtendSlide(),
                                                    armLift.intakeSample()
                                            ),
                                            pickUpSecondSample
                                    ),
                                    armLift.extendSlide(),
                                    new SleepAction(0.25),
                                    clawSystem.closeClaw(),
                                    depositSecondSample,
                                    armLift.depositSample(),
                                    armLift.extendSlide(),
                                    new SleepAction(0.5),
                                    clawSystem.openClaw(),

                                    new SleepAction(0.5),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    new SleepAction(0.25),
                                                    armLift.deExtendSlide(),
                                                    armLift.intakeSample()
                                            ),
                                            pickUpThirdSample
                                    ),
                                    armLift.extendSlide(),
                                    new SleepAction(0.25),
                                    clawSystem.closeClaw(),
                                    depositThirdSample,
                                    armLift.depositSample(),
                                    armLift.extendSlide(),
                                    new SleepAction(0.5),
                                    clawSystem.openClaw(),

                                    new SleepAction(0.5),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    new SleepAction(0.25),
                                                    armLift.deExtendSlide(),
                                                    armLift.intakeSample()
                                            ),
                                            pickUpFourthSample
                                    ),
                                    armLift.extendSlide(),
                                    new SleepAction(0.25),
                                    clawSystem.closeClaw(),
                                    depositFourthSample,
                                    armLift.depositSample(),
                                    armLift.extendSlide(),
                                    new SleepAction(0.5),
                                    clawSystem.openClaw(),
                                    armLift.deExtendSlide(),
                                    armLift.restArm(),
                                    armLift.stopUpdate()
                            )
                    )
            );
        }else {

        }
    }

    public void samplePaths() {

    }
    public void specimenPaths() {
        TrajectoryActionBuilder depositFirstSampleB = drive.actionBuilder(specimenStart)
                .setTangent(Math.toRadians(90)) // beginning tangent
                .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(45)), Math.toRadians(225)); // end tangent
        TrajectoryActionBuilder pickUpSecondSampleB = depositFirstSampleB.endTrajectory().fresh()
                .setTangent(Math.toRadians(45)) // beginning tangent
                .splineToLinearHeading(new Pose2d(-48.25, -40, Math.toRadians(90)), Math.toRadians(90)); // end tangent
        TrajectoryActionBuilder depositSecondSampleB = pickUpSecondSampleB.endTrajectory().fresh()
                .setTangent(Math.toRadians(270)) // beginning tangent
                .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(45)), Math.toRadians(225)); // end tangent
        TrajectoryActionBuilder pickUpThirdSampleB = depositSecondSampleB.endTrajectory().fresh()
                .setTangent(Math.toRadians(45)) // beginning tangent
                .splineToLinearHeading(new Pose2d(-38.25, -40, Math.toRadians(90)), Math.toRadians(90)); // end tangent
        TrajectoryActionBuilder depositThirdSampleB = pickUpThirdSampleB.endTrajectory().fresh()
                .setTangent(Math.toRadians(270)) // beginning tangent
                .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(45)), Math.toRadians(225));
        TrajectoryActionBuilder pickUpFourthSampleB = depositThirdSampleB.endTrajectory().fresh()
                .setTangent(Math.toRadians(45)) // beginning tangent
                .splineToLinearHeading(new Pose2d(-64, -24, Math.toRadians(180)), Math.toRadians(180)); // end tangent
        TrajectoryActionBuilder depositFourthSampleB = pickUpFourthSampleB.endTrajectory().fresh()
                .setTangent(Math.toRadians(270)) // beginning tangent
                .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(45)), Math.toRadians(225));

//        TrajectoryActionBuilder parkingB = depositFourthSampleB.endTrajectory().fresh()
//                ;

        depositFirstSample = depositFirstSampleB.build();
        pickUpSecondSample = pickUpSecondSampleB.build();
        depositSecondSample = depositSecondSampleB.build();
        pickUpThirdSample = pickUpThirdSampleB.build();
        depositThirdSample = depositThirdSampleB.build();
        pickUpFourthSample = pickUpFourthSampleB.build();
        depositFourthSample = depositFourthSampleB.build();

    }
}
