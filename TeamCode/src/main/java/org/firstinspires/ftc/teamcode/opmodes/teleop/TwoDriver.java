package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
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

    private double intakeSampleTime = 0;

    private boolean reversed = false;

    private boolean oneSidedLeft = false;
    private boolean oneSidedRight = false;

    private boolean leftClosed = true;
    private boolean rightClosed = true;

    private enum ArmControl {
        GOING_TO_REST,
        GOING_TO_REST2,
        GOING_TO_REST3,
        REST,
        OFF,
        MOVE_ARM,
        INTAKE_SAMPLE_DEFAULT,
        INTAKE_SAMPLE1,
        INTAKE_SAMPLE2,
        INTAKE_SAMPLE3,
        INTAKE_SPECIMEN_DEFAULT,
        INTAKE_SPECIMEN1,
        INTAKE_SPECIMEN2,
        INTAKE_SPECIMEN3,
        DEPOSIT_SPECIMEN_DEFAULT
    }
    private ArmControl armControl = ArmControl.REST;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotSettings.SUPER_TIME.reset();

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        hwMap = hardwareMap;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hwMap, telemetry);

        robot.slideSubsystem.slideState = LinearSlide.SlideState.BASIC_PID;

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
                clawControls();
                armControls();

                telemetry.addData("ARM STATE", armControl);

                robot.update(true, true);
            }
        }

    }

    public void clawControls() {
        if (BulkReading.pMotorArmR > 2750) {
            reversed = true;
        }else {
            reversed = false;
        }
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            if (reversed) {
                rightClosed = !rightClosed;
            }else {
                leftClosed = !leftClosed;
            }
        }
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            if (reversed) {
                leftClosed = !leftClosed;
            }else {
                rightClosed = !rightClosed;
            }
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
            x /= 2.25;
            y /= 2.25;
            r /= 2.25;
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
                    armControl = ArmControl.DEPOSIT_SPECIMEN_DEFAULT;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSample(true);
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSpecimen(true);
                    armControl = ArmControl.INTAKE_SPECIMEN_DEFAULT;
                }

                if (currentGamepad2.right_trigger > 0.01 || currentGamepad2.left_trigger > 0.01) {
                    armControl = ArmControl.MOVE_ARM;
                }
                if (Math.abs(currentGamepad2.right_stick_y) > 0.01 || Math.abs(currentGamepad2.left_stick_y) > 0.01) {
                    armControl = ArmControl.MOVE_ARM;
                }
                break;
            case MOVE_ARM:
                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    leftClosed = true;
                    rightClosed = true;
                    robot.armSubsystem.setRest();
                    armControl = ArmControl.GOING_TO_REST;
                }
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.armSubsystem.setDepositSample(true);
                }
                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.armSubsystem.setDepositSpecimen(true);
                    armControl = ArmControl.DEPOSIT_SPECIMEN_DEFAULT;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSample(true);
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSpecimen(true);
                    armControl = ArmControl.INTAKE_SPECIMEN_DEFAULT;
                }

                if (currentGamepad2.right_trigger > 0.01 && currentGamepad2.left_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() + Arm.pivotSpeedConstant * currentGamepad2.right_trigger;
                    if (newPosition < Arm.SERVO_LIMIT) {
                        newPosition = Arm.SERVO_LIMIT;
                    }
                    robot.armSubsystem.setPivot(newPosition);
                }
                if (currentGamepad2.left_trigger > 0.01 && currentGamepad2.right_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() - Arm.pivotSpeedConstant * currentGamepad2.left_trigger;
                    if (newPosition < Arm.SERVO_LIMIT) {
                        newPosition = Arm.SERVO_LIMIT;
                    }
                    robot.armSubsystem.setPivot(newPosition);
                }

                if (Math.abs(currentGamepad2.right_stick_y) > 0.01) {
                    if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE || robot.armSubsystem.armState == Arm.ArmState.AT_REST) {
                        robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
                        robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    }else if (robot.armSubsystem.armState == Arm.ArmState.BASIC_PID) {
                        robot.armSubsystem.referencePos = robot.armSubsystem.referencePos + Arm.armSpeedConstant * currentGamepad2.right_stick_y * -1;
                    }
                }
                break;
            case DEPOSIT_SPECIMEN_DEFAULT:
                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    leftClosed = true;
                    rightClosed = true;
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
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSample(true);
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSpecimen(true);
                    armControl = ArmControl.INTAKE_SPECIMEN_DEFAULT;
                }

                // shoot arm up
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    Arm.referencePos = Arm.armPresetDepositSpecimenAuto;
                }

                if (currentGamepad2.right_trigger > 0.01 && currentGamepad2.left_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() + Arm.pivotSpeedConstant * currentGamepad2.right_trigger;
                    if (newPosition < Arm.SERVO_LIMIT) {
                        newPosition = Arm.SERVO_LIMIT;
                    }
                    robot.armSubsystem.setPivot(newPosition);
                }
                if (currentGamepad2.left_trigger > 0.01 && currentGamepad2.right_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() - Arm.pivotSpeedConstant * currentGamepad2.left_trigger;
                    if (newPosition < Arm.SERVO_LIMIT) {
                        newPosition = Arm.SERVO_LIMIT;
                    }
                    robot.armSubsystem.setPivot(newPosition);
                }

                if (Math.abs(currentGamepad2.right_stick_y) > 0.01) {
                    if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE || robot.armSubsystem.armState == Arm.ArmState.AT_REST) {
                        robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
                        robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    }else if (robot.armSubsystem.armState == Arm.ArmState.BASIC_PID) {
                        robot.armSubsystem.referencePos = robot.armSubsystem.referencePos + Arm.armSpeedConstant * currentGamepad2.right_stick_y * -1;
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
            case INTAKE_SAMPLE_DEFAULT:
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    intakeSampleTime = RobotSettings.SUPER_TIME.seconds();
                    armControl = ArmControl.INTAKE_SAMPLE1;
                    oneSidedLeft = true;
                    oneSidedRight = true;
                    leftClosed = false;
                    rightClosed = false;
                }
                if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                    intakeSampleTime = RobotSettings.SUPER_TIME.seconds();
                    armControl = ArmControl.INTAKE_SAMPLE1;
                    oneSidedLeft = false;
                    oneSidedRight = true;
                    leftClosed = false;
                    rightClosed = false;
                }
                if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                    intakeSampleTime = RobotSettings.SUPER_TIME.seconds();
                    armControl = ArmControl.INTAKE_SAMPLE1;
                    oneSidedLeft = true;
                    oneSidedRight = false;
                    leftClosed = false;
                    rightClosed = false;
                }
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.armSubsystem.setDepositSample(true);
                    armControl = ArmControl.MOVE_ARM;
                }
                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.armSubsystem.setDepositSpecimen(true);
                    armControl = ArmControl.DEPOSIT_SPECIMEN_DEFAULT;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSpecimen(true);
                    armControl = ArmControl.INTAKE_SPECIMEN_DEFAULT;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSample(true);
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    leftClosed = true;
                    rightClosed = true;
                    robot.armSubsystem.setRest();
                    armControl = ArmControl.GOING_TO_REST;
                }
                if (currentGamepad2.right_trigger > 0.01 && currentGamepad2.left_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() + Arm.pivotSpeedConstant * currentGamepad2.right_trigger;
                    if (newPosition < Arm.SERVO_LIMIT) {
                        newPosition = Arm.SERVO_LIMIT;
                    }
                    robot.armSubsystem.setPivot(newPosition);
                }
                if (currentGamepad2.left_trigger > 0.01 && currentGamepad2.right_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() - Arm.pivotSpeedConstant * currentGamepad2.left_trigger;
                    if (newPosition < Arm.SERVO_LIMIT) {
                        newPosition = Arm.SERVO_LIMIT;
                    }
                    robot.armSubsystem.setPivot(newPosition);
                }
                if (Math.abs(currentGamepad2.right_stick_y) > 0.01) {
                    if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE || robot.armSubsystem.armState == Arm.ArmState.AT_REST) {
                        robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
                        robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    }else if (robot.armSubsystem.armState == Arm.ArmState.BASIC_PID) {
                        robot.armSubsystem.referencePos = robot.armSubsystem.referencePos + Arm.armSpeedConstant * currentGamepad2.right_stick_y * -1;
                    }
                }
                break;
            case INTAKE_SAMPLE1:
                if (RobotSettings.SUPER_TIME.seconds() - intakeSampleTime > 0.25) {
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
//                    Arm.referencePos = Arm.armPresetIntakeSampleAuto;
                    Arm.referencePos = BulkReading.pMotorArmR + Arm.armLowerConstant;
                    armControl = ArmControl.INTAKE_SAMPLE2;
                }
                break;
            case INTAKE_SAMPLE2:
                if (RobotSettings.SUPER_TIME.seconds() - intakeSampleTime > 0.4) {
                    leftClosed = oneSidedLeft;
                    rightClosed = oneSidedRight;
                    armControl = ArmControl.INTAKE_SAMPLE3;
                }
                break;
            case INTAKE_SAMPLE3:
                if (RobotSettings.SUPER_TIME.seconds() - intakeSampleTime > 0.85) {
                    Arm.referencePos = Arm.armPresetIntakeSample;
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                break;
            case INTAKE_SPECIMEN_DEFAULT:
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    intakeSampleTime = RobotSettings.SUPER_TIME.seconds();
                    armControl = ArmControl.INTAKE_SPECIMEN1;
                    oneSidedLeft = true;
                    oneSidedRight = true;
                    leftClosed = false;
                    rightClosed = false;
                }
                if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                    intakeSampleTime = RobotSettings.SUPER_TIME.seconds();
                    armControl = ArmControl.INTAKE_SPECIMEN1;
                    oneSidedLeft = false;
                    oneSidedRight = true;
                    leftClosed = false;
                    rightClosed = false;
                }
                if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                    intakeSampleTime = RobotSettings.SUPER_TIME.seconds();
                    armControl = ArmControl.INTAKE_SPECIMEN1;
                    oneSidedLeft = true;
                    oneSidedRight = false;
                    leftClosed = false;
                    rightClosed = false;
                }
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.armSubsystem.setDepositSample(true);
                    armControl = ArmControl.MOVE_ARM;
                }
                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.armSubsystem.setDepositSpecimen(true);
                    armControl = ArmControl.DEPOSIT_SPECIMEN_DEFAULT;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSpecimen(true);
                    armControl = ArmControl.INTAKE_SPECIMEN_DEFAULT;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSample(true);
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    leftClosed = true;
                    rightClosed = true;
                    robot.armSubsystem.setRest();
                    armControl = ArmControl.GOING_TO_REST;
                }
                if (currentGamepad2.right_trigger > 0.01 && currentGamepad2.left_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() + Arm.pivotSpeedConstant * currentGamepad2.right_trigger;
                    if (newPosition < Arm.SERVO_LIMIT) {
                        newPosition = Arm.SERVO_LIMIT;
                    }
                    robot.armSubsystem.setPivot(newPosition);
                }
                if (currentGamepad2.left_trigger > 0.01 && currentGamepad2.right_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() - Arm.pivotSpeedConstant * currentGamepad2.left_trigger;
                    if (newPosition < Arm.SERVO_LIMIT) {
                        newPosition = Arm.SERVO_LIMIT;
                    }
                    robot.armSubsystem.setPivot(newPosition);
                }
                if (Math.abs(currentGamepad2.right_stick_y) > 0.01) {
                    if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE || robot.armSubsystem.armState == Arm.ArmState.AT_REST) {
                        robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
                        robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    }else if (robot.armSubsystem.armState == Arm.ArmState.BASIC_PID) {
                        robot.armSubsystem.referencePos = robot.armSubsystem.referencePos + Arm.armSpeedConstant * currentGamepad2.right_stick_y * -1;
                    }
                }
                break;
            case INTAKE_SPECIMEN1:
                if (RobotSettings.SUPER_TIME.seconds() - intakeSampleTime > 0.25) {
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
//                    Arm.referencePos = Arm.armPresetIntakeSpecimenAuto;
                    Arm.referencePos = BulkReading.pMotorArmR + Arm.armLowerConstant;
                    armControl = ArmControl.INTAKE_SPECIMEN2;
                }
                break;
            case INTAKE_SPECIMEN2:
                if (RobotSettings.SUPER_TIME.seconds() - intakeSampleTime > 0.4) {
                    leftClosed = oneSidedLeft;
                    rightClosed = oneSidedRight;
                    armControl = ArmControl.INTAKE_SPECIMEN3;
                }
                break;
            case INTAKE_SPECIMEN3:
                if (RobotSettings.SUPER_TIME.seconds() - intakeSampleTime > 0.85) {
                    Arm.referencePos = Arm.armPresetIntakeSpecimen;
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    armControl = ArmControl.INTAKE_SPECIMEN_DEFAULT;
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
