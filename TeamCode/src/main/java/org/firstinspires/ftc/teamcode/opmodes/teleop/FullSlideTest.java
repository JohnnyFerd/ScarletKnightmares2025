package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.util.BulkReading;


@Config
@TeleOp(name = "Full Slide Test", group = "Testing")
public class FullSlideTest extends LinearOpMode {

    private Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;
    private JVBoysSoccerRobot robot;
    public static double GOAL_POSITION_ARM = 0;
    public static int ACL = 400, VEL = 400, DCL = 200;
    public static double GOAL_POSITION_LINKAGE = 0;
    public static int ACL_L = 400, VEL_L = 400, DCL_L = 200;

    public enum ArmTestState {
        OFF,
        PID_TO_POSITION,
        MOTION_PROFILE
    }
    private ArmTestState armTestState = ArmTestState.OFF;

    public enum SlideTestState {
        OFF,
        PID_TO_POSITION,
        MOTION_PROFILE
    }
    private SlideTestState slideTestState = SlideTestState.OFF;

    @Override
    public void runOpMode() throws InterruptedException {
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.clear();
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                armControls();
                slideControls();

                telemetry.addData("Arm State", armTestState);
                telemetry.addData("Slide State", slideTestState);
                telemetry.addData("Encoder Value", BulkReading.pMotorArmR);
                telemetry.addData("Goal Position", GOAL_POSITION_ARM);

                robot.update(true, true);
            }
        }
    }

    public void armControls() {
        switch (armTestState) {
            case OFF:
                if (currentGamepad1.x && !previousGamepad1.x) {
                    armTestState = ArmTestState.PID_TO_POSITION;
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                }
                if (currentGamepad1.y && !previousGamepad1.y) {
                    armTestState = ArmTestState.MOTION_PROFILE;
                    robot.armSubsystem.setMotionProfile((int) GOAL_POSITION_ARM, ACL, VEL, DCL);
                }
                robot.armSubsystem.referencePos = GOAL_POSITION_ARM;
                break;
            case PID_TO_POSITION:
                if (currentGamepad1.x && !previousGamepad1.x) {
                    armTestState = ArmTestState.OFF;
                    robot.armSubsystem.armState = Arm.ArmState.AT_REST;
                }
                robot.armSubsystem.referencePos = GOAL_POSITION_ARM;
                break;
            case MOTION_PROFILE:
                if (currentGamepad1.y && !previousGamepad1.y) {
                    armTestState = ArmTestState.OFF;
                    robot.armSubsystem.armState = Arm.ArmState.AT_REST;
                }
                if (currentGamepad1.x && !previousGamepad1.x) {
                    armTestState = ArmTestState.PID_TO_POSITION;
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                }
                robot.armSubsystem.referencePos = GOAL_POSITION_ARM;
                break;
        }
    }
    public void slideControls() {
        switch (slideTestState) {
            case OFF:
                if (currentGamepad1.a && !previousGamepad1.a) {
                    slideTestState = SlideTestState.PID_TO_POSITION;
                    robot.slideSubsystem.slideState = LinearSlide.SlideState.BASIC_PID;
                }
                if (currentGamepad1.b && !previousGamepad1.b) {
                    slideTestState = SlideTestState.MOTION_PROFILE;
                    robot.slideSubsystem.setMotionProfile((int) GOAL_POSITION_LINKAGE, ACL, VEL, DCL);
                }
                robot.slideSubsystem.referencePos = GOAL_POSITION_LINKAGE;
                break;
            case PID_TO_POSITION:
                if (currentGamepad1.a && !previousGamepad1.a) {
                    slideTestState = SlideTestState.OFF;
                    robot.slideSubsystem.slideState = LinearSlide.SlideState.AT_REST;
                }
                robot.slideSubsystem.referencePos = GOAL_POSITION_LINKAGE;
                break;
            case MOTION_PROFILE:
                if (currentGamepad1.b && !previousGamepad1.b) {
                    slideTestState = SlideTestState.OFF;
                    robot.slideSubsystem.slideState = LinearSlide.SlideState.AT_REST;
                }
                if (currentGamepad1.a && !previousGamepad1.a) {
                    slideTestState = SlideTestState.PID_TO_POSITION;
                    robot.slideSubsystem.slideState = LinearSlide.SlideState.BASIC_PID;
                }
                robot.slideSubsystem.referencePos = GOAL_POSITION_LINKAGE;
                break;
        }
    }
}
