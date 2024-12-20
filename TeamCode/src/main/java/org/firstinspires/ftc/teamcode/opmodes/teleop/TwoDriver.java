package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.BulkReading;

@TeleOp (name="TWO DRIVER", group="FINAL")
public class TwoDriver extends LinearOpMode {

    private HardwareMap hwMap;
    private JVBoysSoccerRobot robot;

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Gamepad currentGamepad2;
    private Gamepad previousGamepad2;

    private double previousX = 0, previousY = 0, previousR = 0;
    private double resetTime = 0;

    private boolean leftClosed = true;
    private boolean rightClosed = true;

    private enum ArmControl {
        GOING_TO_REST,
        GOING_TO_REST2,
        GOING_TO_REST3,
        REST,
        OFF,
        MOVE_ARM
    }
    private ArmControl armControl = ArmControl.REST;

    private enum SlideControl {
        REST,
        OFF,
        MOVE_SLIDE
    }
    private SlideControl slideControl = SlideControl.REST;

    @Override
    public void runOpMode() throws InterruptedException {
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        hwMap = hardwareMap;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hwMap, telemetry);

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

                drivetrainControls();
//                clawControls();
                armControls();

                robot.update(true, true);
            }
        }

    }

    public void clawControls() {
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            leftClosed = !leftClosed;
        }
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
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
        double x = gamepad1.left_stick_x * 1.05;
        double y = gamepad1.left_stick_y * -1;
        double r = gamepad1.right_stick_x;

//        if (currentGamepad1.b && !previousGamepad1.b) {
//            robot.drivetrainSubsystem.isFieldCentric = !robot.drivetrainSubsystem.isFieldCentric;
//        }

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            robot.drivetrainSubsystem.resetInitYaw();
        }

        if (currentGamepad1.a && !previousGamepad1.a) {
            robot.drivetrainSubsystem.orthogonalMode = !robot.drivetrainSubsystem.orthogonalMode;
        }

        if (currentGamepad1.right_trigger > 0.01 || currentGamepad1.left_trigger > 0.01) {
            x /= 3;
            y /= 3;
            r /= 3;
        }

        // attempting to save motor calls == faster frequency of command calls
        if ( !(previousX == x && previousY == y && previousR == r) ) {
            robot.drivetrainSubsystem.moveXYR(x, y, r);
        }

        previousX = x;
        previousY = y;
        previousR = r;
    }

    public void armControls() {
        switch (armControl) {
            case REST:
                robot.armSubsystem.setPivotRest();
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.armSubsystem.setDepositSample(true);
                    armControl = ArmControl.MOVE_ARM;
                }
                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.armSubsystem.setDepositSpecimen(true);
                    armControl = ArmControl.MOVE_ARM;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    robot.armSubsystem.setIntakeSample(true);
                    armControl = ArmControl.MOVE_ARM;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    robot.armSubsystem.setIntakeSpecimen(true);
                    armControl = ArmControl.MOVE_ARM;
                }

                if (currentGamepad2.right_trigger > 0.01 && currentGamepad2.left_trigger <= 0.01) {
                    armControl = ArmControl.MOVE_ARM;
                }
                if (currentGamepad2.left_trigger > 0.01 && currentGamepad2.right_trigger <= 0.01) {
                    armControl = ArmControl.MOVE_ARM;
                }
                if (Math.abs(currentGamepad2.right_stick_y) > 0.01) {
                    armControl = ArmControl.MOVE_ARM;
                }
                else if (Math.abs(currentGamepad2.left_stick_y) > 0.01) {
                    armControl = ArmControl.MOVE_ARM;
                }
                break;
            case MOVE_ARM:
                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    robot.clawSubsystem.closeBothClaw();
                    robot.armSubsystem.setRest();
                    armControl = ArmControl.GOING_TO_REST;
                }
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.armSubsystem.setDepositSample(true);
                }
                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.armSubsystem.setDepositSpecimen(true);
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    robot.armSubsystem.setIntakeSample(true);
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    robot.armSubsystem.setIntakeSpecimen(true);
                }

                if (currentGamepad2.right_trigger > 0.01 && currentGamepad2.left_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() + Arm.pivotSpeedConstant * currentGamepad2.right_trigger;
                    robot.armSubsystem.setPivot(newPosition);
                }
                if (currentGamepad2.left_trigger > 0.01 && currentGamepad2.right_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() - Arm.pivotSpeedConstant * currentGamepad2.left_trigger;
                    robot.armSubsystem.setPivot(newPosition);
                }

                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    robot.armSubsystem.pivotDown = !robot.armSubsystem.pivotDown;
                    if (robot.armSubsystem.pivotDown) {
                        robot.armSubsystem.setPivot(robot.armSubsystem.previousPivotPos - Arm.pivotDownIncrement);
                    }else {
                        robot.armSubsystem.setPivot(robot.armSubsystem.previousPivotPos);
                    }
                }

                if (Math.abs(currentGamepad2.right_stick_y) > 0.01) {
                    if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE || robot.armSubsystem.armState == Arm.ArmState.AT_REST) {
                        robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
                        robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    }else if (robot.armSubsystem.armState == Arm.ArmState.BASIC_PID) {
                        robot.armSubsystem.referencePos = robot.armSubsystem.referencePos + Arm.armSpeedConstantBig * currentGamepad2.right_stick_y * -1;
                    }
                }else if (Math.abs(currentGamepad2.left_stick_y) > 0.01) {
                    if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE || robot.armSubsystem.armState == Arm.ArmState.AT_REST) {
                        robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
                        robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    }else if (robot.armSubsystem.armState == Arm.ArmState.BASIC_PID) {
                        robot.armSubsystem.referencePos = robot.armSubsystem.referencePos + Arm.armSpeedConstant * currentGamepad2.left_stick_y * -1;
                    }
                }

                break;
            case OFF:
                break;
            case GOING_TO_REST:
                if (!robot.armSubsystem.getMP().isBusy()) {
                    resetTime = RobotSettings.SUPER_TIME.seconds();
                    armControl = ArmControl.GOING_TO_REST2;
                }
                break;
            case GOING_TO_REST2:
                if (RobotSettings.SUPER_TIME.seconds() - resetTime > 0.2) {
                    robot.armSubsystem.armState = Arm.ArmState.AT_REST;
                    robot.armSubsystem.setArmPower(0);
                    resetTime = RobotSettings.SUPER_TIME.seconds();
                    armControl = ArmControl.GOING_TO_REST3;
                }
                break;
            case GOING_TO_REST3:
                if (RobotSettings.SUPER_TIME.seconds() - resetTime > 0.2) {
                    robot.motorArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motorArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armControl = ArmControl.REST;
                }
                break;
        }
        // EMERGENCY ARM RESET
        if ((currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) || (currentGamepad2.left_stick_button && !previousGamepad2.left_stick_button)) {
            robot.motorArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.armSubsystem.armState = Arm.ArmState.AT_REST;
            armControl = ArmControl.MOVE_ARM;
        }

    }
}
