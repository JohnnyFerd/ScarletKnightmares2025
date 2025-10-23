package org.firstinspires.ftc.teamcode.opmodes.AlanStuff;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.BulkReading;

@Config
@TeleOp(name = "Full Slide Test", group = "Testing")
public class FullSlideTest extends LinearOpMode {

    private Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;
    private JVBoysSoccerRobot robot;
    public static double GOAL_POSITION_ARM = 0;
    public static int ACL = 3000, VEL = 3000, DCL = 1500;

    public static double PIVOT_SERVO_POSITION_1 = 1, PIVOT_SERVO_POSITION_2 = 0;

    private boolean leftClosed = true;
    private boolean rightClosed = true;

    private double previousX = 0, previousY = 0, previousR = 0;

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

    public enum PivotTestState {
        POSITION_1,
        POSITION_2,
        CUSTOM
    }
    private PivotTestState pivotTestState = PivotTestState.POSITION_1;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotSettings.SUPER_TIME.reset();

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
                armPivotControls();
                clawControls();
                drivetrainControls();

                telemetry.addData("Arm Test State", armTestState);
                telemetry.addData("Arm State", robot.armSubsystem.armState);
                telemetry.addData("Slide State", slideTestState);
                telemetry.addData("Pivot State", pivotTestState);
                telemetry.addData("Encoder Value (Arm)", BulkReading.pMotorArmR);
                telemetry.addData("Goal Position (Arm)", GOAL_POSITION_ARM);
                telemetry.addData("Pivot Servo Position (R)", robot.servoPivotR.getPosition());
                telemetry.addData("Pivot Servo Position (L)", robot.servoPivotL.getPosition());
                telemetry.addData("Claw Wrist Position", robot.servoWrist.getPosition());

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

    public void armPivotControls() {
        switch (pivotTestState) {
            case POSITION_1:
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    pivotTestState = PivotTestState.POSITION_2;
                }
                if (currentGamepad1.right_trigger > 0.01 && currentGamepad1.left_trigger <= 0.01) {
                    pivotTestState = PivotTestState.CUSTOM;
                }
                if (currentGamepad1.left_trigger > 0.01 && currentGamepad1.right_trigger <= 0.01) {
                    pivotTestState = PivotTestState.CUSTOM;
                }
                robot.armSubsystem.setPivot(PIVOT_SERVO_POSITION_1);
                break;
            case POSITION_2:
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    pivotTestState = PivotTestState.POSITION_1;
                }
                if (currentGamepad1.right_trigger > 0.01 && currentGamepad1.left_trigger <= 0.01) {
                    pivotTestState = PivotTestState.CUSTOM;
                }
                if (currentGamepad1.left_trigger > 0.01 && currentGamepad1.right_trigger <= 0.01) {
                    pivotTestState = PivotTestState.CUSTOM;
                }
                robot.armSubsystem.setPivot(PIVOT_SERVO_POSITION_2);
                break;
            case CUSTOM:
                if (currentGamepad1.right_trigger > 0.01 && currentGamepad1.left_trigger <= 0.01) {
                    double newPosition = robot.servoPivotL.getPosition() + Arm.pivotSpeedConstant * currentGamepad1.right_trigger;
                    robot.armSubsystem.setPivot(newPosition);
                }
                else if (currentGamepad1.left_trigger > 0.01 && currentGamepad1.right_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() - Arm.pivotSpeedConstant * currentGamepad1.left_trigger;
                    robot.armSubsystem.setPivot(newPosition);
                }

                if (currentGamepad1.left_stick_button) {
                    double newPosition = robot.servoWrist.getPosition() + (Arm.pivotSpeedConstant / 4.0);
                    robot.servoWrist.setPosition(newPosition);
                }else if (currentGamepad1.right_stick_button) {
                    double newPosition = robot.servoWrist.getPosition() - (Arm.pivotSpeedConstant / 4.0);
                    robot.servoWrist.setPosition(newPosition);
                }

                if (currentGamepad1.dpad_up && !currentGamepad1.dpad_up) {
                    pivotTestState = PivotTestState.POSITION_1;
                }
                break;
        }
    }

    public void clawControls() {
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            leftClosed = !leftClosed;
        }
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            rightClosed = !rightClosed;
        }

        if (leftClosed && rightClosed) {
            robot.clawSubsystem.closeBothClaw();
        }
        if (leftClosed && !rightClosed) {
            robot.clawSubsystem.openRightClaw();
        }
        if (rightClosed && !leftClosed) {
            robot.clawSubsystem.openLeftClaw();
        }
        if (!rightClosed && !leftClosed) {
            robot.clawSubsystem.openBothClaw();
        }
    }

    public void drivetrainControls() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y * -1;
        double r = gamepad1.right_stick_x;

        if (currentGamepad1.b && !previousGamepad1.b) {
            robot.drivetrainSubsystem.isFieldCentric = !robot.drivetrainSubsystem.isFieldCentric;
        }

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            robot.drivetrainSubsystem.orthogonalMode = !robot.drivetrainSubsystem.orthogonalMode;
        }

        if (currentGamepad1.right_trigger > 0.01 || currentGamepad1.left_trigger > 0.01) {
            x /= 3;
            y /= 3;
            r /= 3;
        }

        if (robot.drivetrainSubsystem.lastAngle != null) {
            telemetry.addData("LAST ANGLE", robot.drivetrainSubsystem.lastAngle.firstAngle);
        }
        telemetry.addData("CURRENT REFERENCE ANGLE", 1);
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("R", r);

        // attempting to save motor calls == faster frequency of command calls
        if ( !(previousX == x && previousY == y && previousR == r) ) {
            robot.drivetrainSubsystem.moveXYR(x, y, r);
        }

        previousX = x;
        previousY = y;
        previousR = r;
    }
}
