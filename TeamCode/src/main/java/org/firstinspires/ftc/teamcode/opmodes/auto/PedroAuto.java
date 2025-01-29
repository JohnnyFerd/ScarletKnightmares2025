//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathBuilder;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.pedropathing.constants.FConstants;
//import org.firstinspires.ftc.teamcode.pedropathing.constants.LConstants;
//import org.firstinspires.ftc.teamcode.settings.RobotSettings;
//import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
//
//@Config
//@Autonomous(name = "Pedro Specimen Auto", group = "Testing")
//public class PedroAuto extends LinearOpMode {
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//    /** This is the variable where we store the state of our auto.
//     * It is used by the pathUpdate method. */
//    private int pathState;
//
//    /* Create and Define Poses + Paths
//     * Poses are built with three constructors: x, y, and heading (in Radians).
//     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
//     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
//     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
//     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
//     * Lets assume our robot is 17.5 (front to back) by 16.5 inches (side to side) */
//
//    private final Pose startPose = new Pose(8.75, 63.75, Math.toRadians(180));
//    private final Pose score1Pose = new Pose(36, 63.75, Math.toRadians(180));
//    private final Pose score2Pose = new Pose(36, 66, Math.toRadians(180));
//    private final Pose score3Pose = new Pose(36, 61, Math.toRadians(180));
//    private final Pose score4Pose = new Pose(36, 68, Math.toRadians(180));
//    private final Pose push1Pose = new Pose(16, 26, Math.toRadians(180));
//    private final Pose push2Pose = new Pose(16, 15, Math.toRadians(180));
//    private final Pose push3Pose = new Pose(16, 8.25, Math.toRadians(180));
//    private final Pose pickup2Pose = new Pose(10, 36, Math.toRadians(180));
//    private final Pose pickup3Pose = new Pose(10, 36, Math.toRadians(180));
//    private final Pose pickup4Pose = new Pose(10, 36, Math.toRadians(180));
//    private final Pose parkPose = new Pose(10, 10, Math.toRadians(0));
//
//    private PathChain score1, park, push1, push2, push3;
//    private PathChain pickup2, pickup3, pickup4, score2, score3, score4;
//
//    private JVBoysSoccerRobot robot;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//
//        robot = new JVBoysSoccerRobot(hardwareMap, telemetry, true);
//
//        waitForStart();
//
//        opmodeTimer.resetTimer();
//        setPathState(0);
//        telemetry.clear();
//
//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//                // These loop the movements of the robot
//                follower.update();
//                autonomousPathUpdate();
//
//                // Feedback to Driver Hub
//                telemetry.addData("path state", pathState);
//                telemetry.addData("x", follower.getPose().getX());
//                telemetry.addData("y", follower.getPose().getY());
//                telemetry.addData("heading", follower.getPose().getHeading());
//                telemetry.update();
//            }
//        }
//    }
//
//    public void buildPaths() {
//        score1 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Point(startPose),
//                                new Point(score1Pose)
//                        )
//                )
//                .setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading())
//                .build();
//
//        push1 = follower.pathBuilder()
//                .addPath(
//                        // Line 2
//                        new BezierCurve(
//                                new Point(score1Pose),
//                                new Point(0.500, 15.500, Point.CARTESIAN),
//                                new Point(68.000, 52.000, Point.CARTESIAN),
//                                new Point(62.000, 26.000, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .addPath(
//                        // Line 3
//                        new BezierLine(
//                                new Point(62.000, 26.000, Point.CARTESIAN),
//                                new Point(push1Pose)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        push2 = follower.pathBuilder()
//                .addPath(
//                        // Line 4
//                        new BezierCurve(
//                                new Point(push1Pose),
//                                new Point(72.000, 32.500, Point.CARTESIAN),
//                                new Point(62.000, 15.000, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .addPath(
//                        // Line 5
//                        new BezierLine(
//                                new Point(62.000, 15.000, Point.CARTESIAN),
//                                new Point(push2Pose)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .build();
//
//        push3 = follower.pathBuilder()
//                .addPath(
//                        // Line 6
//                        new BezierCurve(
//                                new Point(push2Pose),
//                                new Point(65.000, 18.000, Point.CARTESIAN),
//                                new Point(62.000, 8.250, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .addPath(
//                        // Line 7
//                        new BezierLine(
//                                new Point(62.000, 8.250, Point.CARTESIAN),
//                                new Point(push3Pose)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .build();
//
//        pickup2 = follower.pathBuilder()
//                .addPath(
//                        // Line 8
//                        new BezierCurve(
//                                new Point(push3Pose),
//                                new Point(30.000, 37.000, Point.CARTESIAN),
//                                new Point(pickup2Pose)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        score2 = follower.pathBuilder()
//                .addPath(
//                        // Line 9
//                        new BezierCurve(
//                                new Point(pickup2Pose),
//                                new Point(7.000, 69.000, Point.CARTESIAN),
//                                new Point(score2Pose)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        pickup3 = follower.pathBuilder()
//                .addPath(
//                        // Line 10
//                        new BezierCurve(
//                                new Point(score2Pose),
//                                new Point(38.000, 32.000, Point.CARTESIAN),
//                                new Point(pickup3Pose)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        score3 = follower.pathBuilder()
//                .addPath(
//                        // Line 11
//                        new BezierCurve(
//                                new Point(pickup3Pose),
//                                new Point(7.000, 62.000, Point.CARTESIAN),
//                                new Point(score3Pose)
//                        )
//                )
//                .build();
//
//        pickup4 = follower.pathBuilder()
//                .addPath(
//                        // Line 12
//                        new BezierCurve(
//                                new Point(score3Pose),
//                                new Point(38.000, 34.000, Point.CARTESIAN),
//                                new Point(pickup4Pose)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        score4 = follower.pathBuilder()
//                .addPath(
//                        // Line 13
//                        new BezierCurve(
//                                new Point(pickup4Pose),
//                                new Point(7.000, 70.000, Point.CARTESIAN),
//                                new Point(score4Pose)
//                        )
//                )
//                .build();
//
//        park = follower.pathBuilder()
//                .addPath(
//                        // Line 14
//                        new BezierLine(
//                                new Point(score4Pose),
//                                new Point(parkPose)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
//                .build();
//    }
//
//    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
//     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
//     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
//    public void autonomousPathUpdate() {
//        updateArm();
//
//        switch (pathState) {
//            case 0:
//                follower.followPath(score1);
//                setPathState(1);
//                break;
//            case 1: // score first specimen
//                /* You could check for
//                - Follower State: "if(!follower.isBusy() {}"
//                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//                - Robot Position: "if(follower.getPose().getX() > 36) {}"
//                */
//                if(follower.getPose().getX() > 12) {
//                    robot.armSubsystem.setDepositSpecimen(true);
//                    setPathState(20);
//                }
//                break;
//            case 20:
//                // .isBusy checks if robot position is close (1 inch) from the desired end pose
//                if (!follower.isBusy()) {
//                    robot.clawSubsystem.openBothClaw();
//                    setPathState(21);
//                }
//                break;
//            case 21:
//                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
//                    follower.followPath(push1,true);
//                    setPathState(22);
//                }
//                break;
//            case 22: // pushing first sample
//                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
//                    robot.armSubsystem.setRest();
//                    robot.clawSubsystem.closeBothClaw();
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if(!follower.isBusy()) {
//                    follower.followPath(push2,true);
//                    setPathState(3);
//                }
//                break;
//            case 3: // pushing second sample
//                if(!follower.isBusy()) {
//                    follower.followPath(push3,true);
//                    setPathState(4);
//                }
//                break;
//            case 4: // pushing third sample
//                if(!follower.isBusy()) {
//                    follower.followPath(pickup2,true);
//                    robot.armSubsystem.setIntakeSpecimen(true);
//                    robot.clawSubsystem.openBothClaw();
//                    setPathState(5);
//                }
//                break;
//            case 5: // picking up second specimen
//                if(!follower.isBusy()) {
//                    setPathState(23);
//                }
//                break;
//            case 23:
//                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
//                    robot.clawSubsystem.closeBothClaw();
//                    setPathState(24);
//                }
//                break;
//            case 24:
//                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
//                    robot.armSubsystem.setDepositSpecimen(true);
//                    setPathState(25);
//                }
//                break;
//            case 25:
//                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
//                    follower.followPath(score2,true);
//                    setPathState(6);
//                }
//                break;
//            case 6: // scoring second specimen
//                if(!follower.isBusy()) {
//                    robot.clawSubsystem.openBothClaw();
//                    setPathState(26);
//                }
//                break;
//            case 26:
//                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
//                    follower.followPath(pickup3, true);
//                    setPathState(7);
//                }
//                break;
//            case 7: // picking up third specimen
//                if(!follower.isBusy()) {
//                    follower.followPath(score3,true);
//                    setPathState(8);
//                }
//                break;
//            case 8: // scoring third specimen
//                if(!follower.isBusy()) {
//                    follower.followPath(pickup4,true);
//                    setPathState(8);
//                }
//                break;
//            case 9: // picking up fourth specimen
//                if(!follower.isBusy()) {
//                    follower.followPath(score4,true);
//                    setPathState(8);
//                }
//                break;
//            case 10: // scoring fourth specimen
//                if(!follower.isBusy()) {
//                    follower.followPath(park,true);
//                    setPathState(8);
//                }
//                break;
//            case 11: // parking
//                if (!follower.isBusy()) {
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    public void updateArm() {
//        robot.armSubsystem.update();
//        robot.BR.readAll();
//    }
//
//    /** These change the states of the paths and actions
//     * It will also reset the timers of the individual switches **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//}
