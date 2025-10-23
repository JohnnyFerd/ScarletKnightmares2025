package org.firstinspires.ftc.teamcode.opmodes.AlanStuff;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.BulkReading;

@Config
public abstract class AutoBase extends LinearOpMode {

    protected Gamepad currentGamepad = new Gamepad();
    protected Gamepad previousGamepad = new Gamepad();

    protected Pose2d specimenStart;
    protected Pose2d specimenStart4;
    protected Pose2d sampleStart;

    protected double specimenStartX = 8, specimenStartY = -63, specimenStartHeading = Math.toRadians(270);

    protected JVBoysSoccerRobot robot;
    protected ArmLift armLift;
    protected ClawSystem clawSystem;

    public static int DEPOSIT_SPECIMEN_FIRST = 3930;
    public static int DEPOSIT_SPECIMEN_POS = DEPOSIT_SPECIMEN_FIRST;
    public static int DEPOSIT_SPECIMEN_SECOND = 3930;
    public static int DEPOSIT_SPECIMEN_DOWN = 3430;
    public static int DEPOSIT_SPECIMEN_UP = Arm.armPresetDepositSpecimen + 400;

    public static int ARM_UP = 2780;

    public static int INTAKE_SPECIMEN_POS = 600;
    public static int INTAKE_SPECIMEN_POS_HIGHER = 600;
    public static int DEPOSIT_SAMPLE_POS = Arm.armPresetDepositSample;
    public static int INTAKE_SAMPLE_POS = Arm.armPresetIntakeSample;
    public static double PIVOT_INTAKE_POS = 0.40 - Arm.PIVOT_OFFSET;

    public static double PIVOT_DEPOSIT_SPECIMEN_POS = 0.29 - Arm.PIVOT_OFFSET;

    public static double clawWristAuto45 = 0.755;
    public static double clawWristAuto180 = 0.1;
    public static double clawWristAuto135 = 0.405;

    protected boolean isBlue = false;

    public void initialize() {
        DEPOSIT_SPECIMEN_POS = DEPOSIT_SPECIMEN_FIRST;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry, true);

        specimenStart = new Pose2d(specimenStartX, specimenStartY, specimenStartHeading);
        specimenStart4 = new Pose2d(specimenStartX, specimenStartY, Math.toRadians(90));
        sampleStart = new Pose2d(-24, -63, Math.toRadians(270));
        armLift = new ArmLift();
        clawSystem = new ClawSystem();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();
    }

    public class ArmLift {
        private boolean stopUpdate = false;

        public class UpdateArmSubsystem implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    stopUpdate = false;
                }

                if (stopUpdate) {
                    return false;
                }else {
//                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.update();
                    robot.BR.readAll();
                    telemetryPacket.put("MOTOR POSITION", BulkReading.pMotorArmR);
                    return true;
                }
            }
        }
        public Action updateArmSubsystem() {
            return new UpdateArmSubsystem();
        }

        public class StopUpdate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                stopUpdate = true;
                return false;
            }
        }
        public Action stopUpdate() {
            return new StopUpdate();
        }

        public class DepositSpecimenRamFront implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setDepositSpecimenRamFront(true);
                    initialized = true;
                }
                if (!robot.armSubsystem.getMP().isBusy()) {
                    return false;
                }
                return true;
            }
        }
        public Action depositSpecimenRamFront() {
            return new DepositSpecimenRamFront();
        }

        public class DepositSpecimenRam implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setDepositSpecimenRam(true);
                    initialized = true;
                }
                if (!robot.armSubsystem.getMP().isBusy()) {
                    return false;
                }
                return true;
            }
        }
        public Action depositSpecimenRam() {
            return new DepositSpecimenRam();
        }

        public class DepositSpecimen implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setMotionProfile(DEPOSIT_SPECIMEN_POS);
                    robot.armSubsystem.pivotCounter = 8;
                    initialized = true;
                }
//                robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
//                robot.armSubsystem.update();
                if (!robot.armSubsystem.getMP().isBusy()) {
                    DEPOSIT_SPECIMEN_POS = DEPOSIT_SPECIMEN_SECOND;
                    return false;
                }
                return true;
            }
        }
        public Action depositSpecimen() {
            return new DepositSpecimen();
        }

        public class DepositSpecimenNoPivotTimed implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setMotionProfile(DEPOSIT_SPECIMEN_POS);
                    robot.armSubsystem.setPivotDepositSpecimen();
                    robot.armSubsystem.pivotCounter = 0;
                    initialized = true;
                }
//                robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
//                robot.armSubsystem.update();
                if (!robot.armSubsystem.getMP().isBusy()) {
                    DEPOSIT_SPECIMEN_POS = DEPOSIT_SPECIMEN_SECOND;
                    return false;
                }
                return true;
            }
        }
        public Action depositSpecimenNoPivotTimed() {
            return new DepositSpecimenNoPivotTimed();
        }

        public class ArmUp implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setMotionProfile(ARM_UP);
                    robot.armSubsystem.setPivotIntakeSample();
                    robot.armSubsystem.pivotCounter = 0;
                    initialized = true;
                }
//                robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
//                robot.armSubsystem.update();
                if (!robot.armSubsystem.getMP().isBusy()) {
                    return false;
                }
                return true;
            }
        }
        public Action armUp() {
            return new ArmUp();
        }


        public class DepositSpecimenDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.referencePos = DEPOSIT_SPECIMEN_DOWN;
                robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                return false;
            }
        }
        public Action depositSpecimenDown() {
            return new DepositSpecimenDown();
        }

        public class DepositSpecimenUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.referencePos = DEPOSIT_SPECIMEN_UP;
                robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                return false;
            }
        }
        public Action depositSpecimenUp() {
            return new DepositSpecimenUp();
        }

        public class IntakeSpecimenPivotTimed implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setMotionProfile(INTAKE_SPECIMEN_POS_HIGHER); // TODO: uses higher value
                    robot.armSubsystem.pivotCounter = 1;
                    initialized = true;
                }
                if (!robot.armSubsystem.getMP().isBusy()) {
                    return false;
                }
                return true;
            }
        }
        public Action intakeSpecimenPivotTimed() {
            return new IntakeSpecimenPivotTimed();
        }

        public class IntakeSpecimen implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setMotionProfile(INTAKE_SPECIMEN_POS);
                    robot.armSubsystem.setPivot(PIVOT_INTAKE_POS);
                    initialized = true;
                }
                if (!robot.armSubsystem.getMP().isBusy()) {
                    return false;
                }
                return true;
            }
        }
        public Action intakeSpecimen() {
            return new IntakeSpecimen();
        }

        public class IntakeSpecimenHigher implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setMotionProfile(INTAKE_SPECIMEN_POS_HIGHER);
                    robot.armSubsystem.setPivot(PIVOT_INTAKE_POS);
                    initialized = true;
                }
                if (!robot.armSubsystem.getMP().isBusy()) {
                    return false;
                }
                return true;
            }
        }
        public Action intakeSpecimenHigher() {
            return new IntakeSpecimenHigher();
        }

        public class DepositSample implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setDepositSample(true);
                    initialized = true;
                }
                if (!robot.armSubsystem.getMP().isBusy()) {
                    return false;
                }
                return true;
            }
        }
        public Action depositSample() {
            return new DepositSample();
        }

        public class IntakeSample implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setIntakeSample(false);
                    initialized = true;
                }
                if (!robot.armSubsystem.getMP().isBusy()) {
                    return false;
                }
                return true;
            }
        }
        public Action intakeSample() {
            return new IntakeSample();
        }

        public class IntakeSampleDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                Arm.referencePos = BulkReading.pMotorArmR + Arm.armLowerConstantSampleAuto;
                return false;
            }
        }
        public Action intakeSampleDown() {
            return new IntakeSampleDown();
        }

        public class IntakeSampleUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                Arm.referencePos = Arm.armPresetIntakeSample;
                return false;
            }
        }
        public Action intakeSampleUp() {
            return new IntakeSampleUp();
        }

        public class RestArm implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setRest();
                }
                if (!robot.armSubsystem.getMP().isBusy()) {
                    robot.armSubsystem.armState = Arm.ArmState.AT_REST;
                    robot.armSubsystem.setArmPower(0);
                    robot.motorArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motorArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    return false;
                }
                return true;
            }
        }
        public Action restArm() {
            return new RestArm();
        }

        public class DepositSpecimenFront implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setDepositSpecimenFront(true);
                    initialized = true;
                }
                if (!robot.armSubsystem.getMP().isBusy()) {
                    return false;
                }
                return true;
            }
        }
        public Action depositSpecimenFront() {
            return new DepositSpecimenFront();
        }
        public class DepositSpecimenFrontUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.referencePos = Arm.armPresetDepositSpecimenFrontUp;
                robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                return false;
            }
        }
        public Action depositSpecimenFrontUp() {
            return new DepositSpecimenFrontUp();
        }
    }

    public class ClawSystem {
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Claw.useTightClaw = false;
                robot.clawSubsystem.closeBothClaw();
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class CloseClawTight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Claw.useTightClaw = true;
                robot.clawSubsystem.closeBothClaw();
                return false;
            }
        }
        public Action closeClawTight() {
            return new CloseClawTight();
        }

        public class CloseClawTightest implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Claw.useTightClaw = true;
                robot.clawSubsystem.closeBothClawTightest();
                return false;
            }
        }
        public Action closeClawTightest() {
            return new CloseClawTightest();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.clawSubsystem.openBothClaw();
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }

        public class OpenClawWide implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.clawSubsystem.openBothClawWide();
                return false;
            }
        }
        public Action openClawWide() {
            return new OpenClawWide();
        }

        public class ClawWrist0 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.servoWrist.setPosition(Arm.clawWrist0);
                return false;
            }
        }
        public Action clawWrist0() {
            return new ClawWrist0();
        }

        public class ClawWrist45 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.servoWrist.setPosition(clawWristAuto45);
                return false;
            }
        }
        public Action clawWrist45() {
            return new ClawWrist45();
        }

        public class ClawWrist135 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.servoWrist.setPosition(clawWristAuto135);
                return false;
            }
        }
        public Action clawWrist135() {
            return new ClawWrist135();
        }

        public class ClawWrist180 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.servoWrist.setPosition(clawWristAuto180);
                return false;
            }
        }
        public Action clawWrist180() {
            return new ClawWrist180();
        }
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;

}
