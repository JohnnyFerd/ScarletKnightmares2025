package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.BulkReading;

@Config
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

    public static double pickupTime1 = 0.2, pickupTime2 = 0.25, pickupTime3 = 0.55;

    private double intakeSampleTime = 0;

    private double currentDepositTime = 0;
    private boolean depositDelay = false;

    private boolean reversed = false;

    private boolean oneSidedLeft = false;
    private boolean oneSidedRight = false;

    private boolean leftClosed = true;
    private boolean rightClosed = true;

    private boolean FINAL_RIGGING = false;

    private boolean wrist0 = true;
    private int wristCounter = 1;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private int loopCounter = 0;

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

        Arm.DEFAULT_MAX_ACCELERATION = Arm.TELEOP_MAX_ACCELERATION;
        Arm.DEFAULT_MAX_VELOCITY = Arm.TELEOP_MAX_VELOCITY;
        Arm.DEFAULT_MAX_DECELERATION = Arm.TELEOP_MAX_DECELERATION;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.clear();
            RobotSettings.SUPER_TIME.reset();
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                drivetrainControls();
                clawControls();
                armControls();
                riggingControls();

                telemetry.addData("ARM STATE", armControl);

                loopTimes();

                robot.update(true, true);
            }
        }

    }

    public void loopTimes() {
        loopCounter++;

        telemetry.addData("Loop Time", elapsedTime.milliseconds());
        telemetry.addData("Average Loop Time", RobotSettings.SUPER_TIME.milliseconds() / loopCounter);
        elapsedTime.reset();
    }

    public void riggingControls() {
        if (FINAL_RIGGING) {
            if (currentGamepad1.x) {
                if (currentGamepad1.left_bumper) {
                    robot.motorRigL.setPower(0);
                }else {
                    robot.motorRigL.setPower(RobotSettings.RIGGING_FEEDFORWARD);
                }
                if (currentGamepad1.right_bumper) {
                    robot.motorRigR.setPower(0);
                }else {
                    robot.motorRigR.setPower(RobotSettings.RIGGING_FEEDFORWARD);
                }
            }else {
                if (currentGamepad1.left_bumper) {
                    robot.motorRigL.setPower(RobotSettings.RIGGING_POWER);
                }else {
                    robot.motorRigL.setPower(RobotSettings.RIGGING_FEEDFORWARD);
                }
                if (currentGamepad1.right_bumper) {
                    robot.motorRigR.setPower(RobotSettings.RIGGING_POWER);
                }else {
                    robot.motorRigR.setPower(RobotSettings.RIGGING_FEEDFORWARD);
                }
            }
//            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
//                robot.armSubsystem.setRiggingPosition();
//                armControl = ArmControl.MOVE_ARM;
//            }

        }else {

            if (currentGamepad1.x) {
                if (currentGamepad1.left_bumper) {
                    robot.motorRigL.setPower(-1 * RobotSettings.RIGGING_POWER);
                }else {
                    robot.motorRigL.setPower(0);
                }
                if (currentGamepad1.right_bumper) {
                    robot.motorRigR.setPower(-1 * RobotSettings.RIGGING_POWER);
                }else {
                    robot.motorRigR.setPower(0);
                }
            }else {
                if (currentGamepad1.left_bumper) {
                    robot.motorRigL.setPower(RobotSettings.RIGGING_POWER);
                }else {
                    robot.motorRigL.setPower(0);
                }
                if (currentGamepad1.right_bumper) {
                    robot.motorRigR.setPower(RobotSettings.RIGGING_POWER);
                }else {
                    robot.motorRigR.setPower(0);
                }
            }

        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            FINAL_RIGGING = !FINAL_RIGGING;
        }
    }

    public void clawControls() {
        if (BulkReading.pMotorArmR > RobotSettings.ARM_VERTICAL_THRESHOLD) {
            reversed = true;
        }else {
            reversed = false;
        }
//        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
//            if (reversed) {
//                rightClosed = !rightClosed;
//            }else {
//                leftClosed = !leftClosed;
//            }
//        }
//        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
//            if (reversed) {
//                leftClosed = !leftClosed;
//            }else {
//                rightClosed = !rightClosed;
//            }
//        }
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            rightClosed = !rightClosed;
            leftClosed = !leftClosed;
        }
//        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
//            rightClosed = !rightClosed;
//            leftClosed = !leftClosed;
//        }

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

        if (currentGamepad1.b && !previousGamepad1.b) {
            robot.drivetrainSubsystem.orthogonalMode = !robot.drivetrainSubsystem.orthogonalMode;
        }

        if (currentGamepad1.right_trigger > 0.01 && currentGamepad1.left_trigger > 0.01) {
            x *= 0.3;
            y *= 0.3;
            r *= 0.3;
        }else if (currentGamepad1.right_trigger > 0.01) {
            x *= 0.65;
            y *= 0.65;
            r *= 0.65;
        }else if (currentGamepad1.left_trigger > 0.01) {
            x *= 0.65;
            y *= 0.65;
            r *= 0.65;
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
                robot.servoWrist.setPosition(Arm.clawWrist0);
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    robot.armSubsystem.setDepositSample(true);
                    armControl = ArmControl.MOVE_ARM;
                }
                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.armSubsystem.setDepositSpecimen(true);
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    armControl = ArmControl.DEPOSIT_SPECIMEN_DEFAULT;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    wristCounter = 1;
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    robot.armSubsystem.setIntakeSample(true);
                    Claw.useTightClaw = true;
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
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
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    leftClosed = true;
                    rightClosed = true;
                    robot.armSubsystem.setRest();
                    armControl = ArmControl.GOING_TO_REST;
                }
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    robot.armSubsystem.setDepositSample(true);
                }
                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.armSubsystem.setDepositSpecimen(true);
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    armControl = ArmControl.DEPOSIT_SPECIMEN_DEFAULT;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    wristCounter = 1;
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    robot.armSubsystem.setIntakeSample(true);
                    Claw.useTightClaw = true;
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSpecimen(true);
                    armControl = ArmControl.INTAKE_SPECIMEN_DEFAULT;
                }

                if (currentGamepad2.right_trigger > 0.01 && currentGamepad2.left_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() + Arm.pivotSpeedConstant * currentGamepad2.right_trigger;
                    robot.armSubsystem.setPivot(newPosition);
                }
                if (currentGamepad2.left_trigger > 0.01 && currentGamepad2.right_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() - Arm.pivotSpeedConstant * currentGamepad2.left_trigger;
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
                if (Math.abs(currentGamepad2.left_stick_y) > 0.01) {
                    double newPosition = robot.servoWrist.getPosition() - Arm.wristSpeedConstant * currentGamepad2.left_stick_y;
                    robot.servoWrist.setPosition(newPosition);
                }
                break;
            case DEPOSIT_SPECIMEN_DEFAULT:
                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    leftClosed = true;
                    rightClosed = true;
                    robot.armSubsystem.setRest();
                    armControl = ArmControl.GOING_TO_REST;
                }
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    robot.armSubsystem.setDepositSample(true);
                }
                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    robot.armSubsystem.setDepositSpecimen(true);
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    wristCounter = 1;
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    robot.armSubsystem.setIntakeSample(true);
                    Claw.useTightClaw = true;
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSpecimen(true);
                    armControl = ArmControl.INTAKE_SPECIMEN_DEFAULT;
                }

                // shoot arm up
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    Arm.referencePos = BulkReading.pMotorArmR - Arm.armLowerConstantSpecimen;
                    currentDepositTime = RobotSettings.SUPER_TIME.seconds();
                    armControl = ArmControl.MOVE_ARM;
                    depositDelay = false;
                }
//                if (depositDelay == true && RobotSettings.SUPER_TIME.seconds() > currentDepositTime + Arm.automaticDepositTimeDelay) {
//                    depositDelay = false;
//                    leftClosed = false;
//                    rightClosed = false;
//                }

                if (currentGamepad2.right_trigger > 0.01 && currentGamepad2.left_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() + Arm.pivotSpeedConstant * currentGamepad2.right_trigger;
                    robot.armSubsystem.setPivot(newPosition);
                    Arm.pivotPresetDepositSpecimen = newPosition; // TODO: test this
                }
                if (currentGamepad2.left_trigger > 0.01 && currentGamepad2.right_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() - Arm.pivotSpeedConstant * currentGamepad2.left_trigger;
                    robot.armSubsystem.setPivot(newPosition);
                    Arm.pivotPresetDepositSpecimen = newPosition; // TODO: test this
                }

                if (Math.abs(currentGamepad2.right_stick_y) > 0.01) {
                    if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE || robot.armSubsystem.armState == Arm.ArmState.AT_REST) {
                        robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
                        robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    }else if (robot.armSubsystem.armState == Arm.ArmState.BASIC_PID) {
                        robot.armSubsystem.referencePos = robot.armSubsystem.referencePos + Arm.armSpeedConstant * currentGamepad2.right_stick_y * -1;
                        Arm.armPresetDepositSpecimen = BulkReading.pMotorArmR; // TODO: Test this
                    }
                }
                if (Math.abs(currentGamepad2.left_stick_y) > 0.01) {
                    double newPosition = robot.servoWrist.getPosition() - Arm.wristSpeedConstant * currentGamepad2.left_stick_y;
                    robot.servoWrist.setPosition(newPosition);
                }
                break;
            case OFF:
                break;
            case GOING_TO_REST:
                if (Math.abs(currentGamepad2.right_stick_y) > 0.01 || Math.abs(currentGamepad2.left_stick_y) > 0.01) {
                    robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    armControl = ArmControl.MOVE_ARM;
                }
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
//                if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
//                    intakeSampleTime = RobotSettings.SUPER_TIME.seconds();
//                    armControl = ArmControl.INTAKE_SAMPLE1;
//                    oneSidedLeft = false;
//                    oneSidedRight = true;
//                    leftClosed = false;
//                    rightClosed = false;
//                }
//                if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
//                    intakeSampleTime = RobotSettings.SUPER_TIME.seconds();
//                    armControl = ArmControl.INTAKE_SAMPLE1;
//                    oneSidedLeft = true;
//                    oneSidedRight = false;
//                    leftClosed = false;
//                    rightClosed = false;
//                }
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    robot.armSubsystem.setDepositSample(true);
                    armControl = ArmControl.MOVE_ARM;
                    wrist0 = true;
                    Claw.useTightClaw = false;
                }
                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.armSubsystem.setDepositSpecimen(true);
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    armControl = ArmControl.DEPOSIT_SPECIMEN_DEFAULT;
                    wrist0 = true;
                    Claw.useTightClaw = false;
                }
                if (currentGamepad2.b && !previousGamepad2.b) { // TODO: fix this??? this button doesn't work???
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSpecimen(true);
                    armControl = ArmControl.INTAKE_SPECIMEN_DEFAULT;
                    wrist0 = true;
                    Claw.useTightClaw = false;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    wristCounter = 1;
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    robot.armSubsystem.setIntakeSample(true);
                    Claw.useTightClaw = true;
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    leftClosed = true;
                    rightClosed = true;
                    robot.armSubsystem.setRest();
                    armControl = ArmControl.GOING_TO_REST;
                    wrist0 = true;
                    Claw.useTightClaw = false;
                }
                if (currentGamepad2.right_trigger > 0.01 && currentGamepad2.left_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() + Arm.pivotSpeedConstant * currentGamepad2.right_trigger;
                    robot.armSubsystem.setPivot(newPosition);
                    Arm.pivotPresetIntakeSample = newPosition; // TODO: test this
                }
                if (currentGamepad2.left_trigger > 0.01 && currentGamepad2.right_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() - Arm.pivotSpeedConstant * currentGamepad2.left_trigger;
                    robot.armSubsystem.setPivot(newPosition);
                    Arm.pivotPresetIntakeSample = newPosition; // TODO: test this
                }
                if (Math.abs(currentGamepad2.right_stick_y) > 0.01) {
                    if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE || robot.armSubsystem.armState == Arm.ArmState.AT_REST) {
                        robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
                        robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    }else if (robot.armSubsystem.armState == Arm.ArmState.BASIC_PID) {
                        robot.armSubsystem.referencePos = robot.armSubsystem.referencePos + Arm.armSpeedConstant * currentGamepad2.right_stick_y * -1;
                        Arm.armPresetIntakeSpecimen = BulkReading.pMotorArmR; // TODO: Test this
                    }
                }
                if (Math.abs(currentGamepad2.left_stick_y) > 0.01) {
                    double newPosition = robot.servoWrist.getPosition() - Arm.wristSpeedConstant * currentGamepad2.left_stick_y;
                    robot.servoWrist.setPosition(newPosition);
                }
                if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                    wristCounter++;

                    if (wristCounter % 3 == 1) {
                        robot.servoWrist.setPosition(Arm.clawWrist0);
                    }else if (wristCounter % 3 == 2) {
                        robot.servoWrist.setPosition(Arm.clawWrist45);
                    }else if (wristCounter % 3 == 0){
                        robot.servoWrist.setPosition(Arm.clawWrist90);
                    }
                }
                break;
            case INTAKE_SAMPLE1:
                if (RobotSettings.SUPER_TIME.seconds() - intakeSampleTime > pickupTime1) {
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
//                    Arm.referencePos = Arm.armPresetIntakeSampleAuto;
                    Arm.referencePos = BulkReading.pMotorArmR + Arm.armLowerConstantSample;
                    armControl = ArmControl.INTAKE_SAMPLE2;
                }
                break;
            case INTAKE_SAMPLE2:
                if (RobotSettings.SUPER_TIME.seconds() - intakeSampleTime > pickupTime2) {
                    leftClosed = oneSidedLeft;
                    rightClosed = oneSidedRight;
                    armControl = ArmControl.INTAKE_SAMPLE3;
                }
                break;
            case INTAKE_SAMPLE3:
                if (RobotSettings.SUPER_TIME.seconds() - intakeSampleTime > pickupTime3) {
                    Arm.referencePos = Arm.armPresetIntakeSample;
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                break;
            case INTAKE_SPECIMEN_DEFAULT:
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    robot.armSubsystem.setDepositSample(true);
                    armControl = ArmControl.MOVE_ARM;
                }
                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.armSubsystem.setDepositSpecimen(true);
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    armControl = ArmControl.DEPOSIT_SPECIMEN_DEFAULT;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    leftClosed = false;
                    rightClosed = false;
                    robot.armSubsystem.setIntakeSpecimen(true);
                    armControl = ArmControl.INTAKE_SPECIMEN_DEFAULT;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    wristCounter = 1;
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    robot.armSubsystem.setIntakeSample(true);
                    Claw.useTightClaw = true;
                    armControl = ArmControl.INTAKE_SAMPLE_DEFAULT;
                }
                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    robot.servoWrist.setPosition(Arm.clawWrist0);
                    leftClosed = true;
                    rightClosed = true;
                    robot.armSubsystem.setRest();
                    armControl = ArmControl.GOING_TO_REST;
                }
                if (currentGamepad2.right_trigger > 0.01 && currentGamepad2.left_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() + Arm.pivotSpeedConstant * currentGamepad2.right_trigger;
                    robot.armSubsystem.setPivot(newPosition);
                    Arm.pivotPresetIntakeSpecimen = newPosition;
                }
                if (currentGamepad2.left_trigger > 0.01 && currentGamepad2.right_trigger <= 0.01) {
                    double newPosition = robot.servoPivotR.getPosition() - Arm.pivotSpeedConstant * currentGamepad2.left_trigger;
                    robot.armSubsystem.setPivot(newPosition);
                    Arm.pivotPresetIntakeSpecimen = newPosition; // TODO: test this
                }
                if (Math.abs(currentGamepad2.right_stick_y) > 0.01) {
                    if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE || robot.armSubsystem.armState == Arm.ArmState.AT_REST) {
                        robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
                        robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    }else if (robot.armSubsystem.armState == Arm.ArmState.BASIC_PID) {
                        robot.armSubsystem.referencePos = robot.armSubsystem.referencePos + Arm.armSpeedConstant * currentGamepad2.right_stick_y * -1;
                        Arm.armPresetIntakeSpecimen = (int)robot.armSubsystem.referencePos;
                    }
                }
                if (Math.abs(currentGamepad2.left_stick_y) > 0.01) {
                    double newPosition = robot.servoWrist.getPosition() - Arm.wristSpeedConstant * currentGamepad2.left_stick_y;
                    robot.servoWrist.setPosition(newPosition);
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
