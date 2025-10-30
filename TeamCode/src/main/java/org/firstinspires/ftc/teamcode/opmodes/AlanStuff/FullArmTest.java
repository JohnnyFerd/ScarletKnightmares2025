//package org.firstinspires.ftc.teamcode.opmodes.AlanStuff;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.settings.RobotSettings;
//import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
//
//@Disabled
//@TeleOp (name = "Full Arm Test OLD", group = "Testing")
//public class FullArmTest extends LinearOpMode {
//
//    private HardwareMap hwMap;
//    private JVBoysSoccerRobot robot;
//
//    private Gamepad currentGamepad1;
//    private Gamepad previousGamepad1;
//    private Gamepad currentGamepad2;
//    private Gamepad previousGamepad2;
//
//    private enum TestState {
//        GOING_TO_REST,
//        GOING_TO_REST2,
//        GOING_TO_REST3,
//        REST,
//        OFF,
//        MOVE_ARM
//    }
//
//    private TestState testState = TestState.REST;
//    private double resetTime = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        currentGamepad1 = new Gamepad();
//        previousGamepad1 = new Gamepad();
//        currentGamepad2 = new Gamepad();
//        previousGamepad2 = new Gamepad();
//
//        hwMap = hardwareMap;
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        robot = new JVBoysSoccerRobot(hwMap, telemetry);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
//        telemetry.update();
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//                previousGamepad1.copy(currentGamepad1);
//                currentGamepad1.copy(gamepad1);
//                previousGamepad2.copy(currentGamepad2);
//                currentGamepad2.copy(gamepad2);
//
//                telemetry.addLine("CONTROLS: ");
//                telemetry.addLine("    DPAD UP: Turn motors on / off ");
//                telemetry.addData("    STATE", testState);
//                armControls();
//                clawControls();
//
//                robot.addTelemetry();
//                telemetry.update();
//                robot.armSubsystem.update();
//                robot.BR.readAll();
//            }
//        }
//    }
//
//    public void armControls() {
////        switch (testState) {
////            case REST:
//////                robot.armSubsystem.armState = Arm.ArmState.AT_REST;
////                robot.armSubsystem.setPivotRest();
////                if (currentGamepad1.x && !previousGamepad1.x) {
////                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
////                    robot.armSubsystem.setDepositSample();
////                    testState = TestState.MOVE_ARM;
////                }
////                if (currentGamepad1.y && !previousGamepad1.y) {
////                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
////                    robot.armSubsystem.setDepositSpecimen();
////                    testState = TestState.MOVE_ARM;
////                }
////                if (currentGamepad1.a && !previousGamepad1.a) {
////                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
////                    robot.armSubsystem.setIntakeSample();
////                    testState = TestState.MOVE_ARM;
////                }
////                if (currentGamepad1.b && !previousGamepad1.b) {
////                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
////                    robot.armSubsystem.setIntakeSpecimen();
////                    testState = TestState.MOVE_ARM;
////                }
////                break;
////            case MOVE_ARM:
////                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
////                    robot.armSubsystem.setRest();
////                    testState = TestState.GOING_TO_REST;
////                }
////                if (currentGamepad1.x && !previousGamepad1.x) {
////                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
////                    robot.armSubsystem.setDepositSample();
////                }
////                if (currentGamepad1.y && !previousGamepad1.y) {
////                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
////                    robot.armSubsystem.setDepositSpecimen();
////                }
////                if (currentGamepad1.a && !previousGamepad1.a) {
////                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
////                    robot.armSubsystem.setIntakeSample();
////                }
////                if (currentGamepad1.b && !previousGamepad1.b) {
////                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
////                    robot.armSubsystem.setIntakeSpecimen();
////                }
////
////                if (currentGamepad1.right_trigger > 0.01 && currentGamepad1.left_trigger <= 0.01) {
////                    double newPosition = robot.servoPivotR.getPosition() + Arm.pivotSpeedConstant * currentGamepad1.right_trigger;
////                    robot.armSubsystem.setPivot(newPosition);
////                }
////                if (currentGamepad1.left_trigger > 0.01 && currentGamepad1.right_trigger <= 0.01) {
////                    double newPosition = robot.servoPivotR.getPosition() - Arm.pivotSpeedConstant * currentGamepad1.left_trigger;
////                    robot.armSubsystem.setPivot(newPosition);
////                }
////
////                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
////                    robot.armSubsystem.pivotDown = !robot.armSubsystem.pivotDown;
////                    if (robot.armSubsystem.pivotDown) {
////                        robot.armSubsystem.setPivot(robot.armSubsystem.previousPivotPos - Arm.pivotDownIncrement);
////                    }else {
////                        robot.armSubsystem.setPivot(robot.armSubsystem.previousPivotPos);
////                    }
////                }
////
////                if (Math.abs(currentGamepad1.right_stick_y) > 0.01) {
////                    if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE) {
////                        robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
////                        robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
////                    }else if (robot.armSubsystem.armState == Arm.ArmState.BASIC_PID) {
////                        robot.armSubsystem.referencePos = robot.armSubsystem.referencePos + Arm.armSpeedConstantBig * currentGamepad1.right_stick_y * -1;
////                    }
////                }else if (Math.abs(currentGamepad1.left_stick_y) > 0.01) {
////                    if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE) {
////                        robot.armSubsystem.referencePos = BulkReading.pMotorArmR;
////                        robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
////                    }else if (robot.armSubsystem.armState == Arm.ArmState.BASIC_PID) {
////                        robot.armSubsystem.referencePos = robot.armSubsystem.referencePos + Arm.armSpeedConstant * currentGamepad1.right_stick_y * -1;
////                    }
////                }
////
////                break;
////            case OFF:
////                break;
////            case GOING_TO_REST:
////                if (!robot.armSubsystem.getMP().isBusy()) {
////                    resetTime = runtime.seconds();
////                    testState = TestState.GOING_TO_REST2;
////                }
////                break;
////            case GOING_TO_REST2:
////                if (runtime.seconds() - resetTime > 0.2) {
////                    robot.armSubsystem.armState = Arm.ArmState.AT_REST;
////                    robot.armSubsystem.setArmPower(0);
////                    resetTime = runtime.seconds();
////                    testState = TestState.GOING_TO_REST3;
////                }
////                break;
////            case GOING_TO_REST3:
////                if (runtime.seconds() - resetTime > 0.2) {
////                    robot.motorArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////                    robot.motorArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////                    testState = TestState.REST;
////                }
////                break;
////            }
//    }
//    public void clawControls() {
//
////        if ((currentGamepad1.left_bumper && !previousGamepad1.left_bumper) || (currentGamepad1.right_bumper && !previousGamepad1.right_bumper)) {
////            robot.clawSubsystem.opened = !robot.clawSubsystem.opened;
////        }
////        if (robot.clawSubsystem.opened) {
////            robot.clawSubsystem.openClaw();
////        }else {
////            robot.clawSubsystem.closeClaw();
////        }
//
//    }
//}
