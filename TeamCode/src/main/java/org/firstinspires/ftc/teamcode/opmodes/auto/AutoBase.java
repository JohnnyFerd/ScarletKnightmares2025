package org.firstinspires.ftc.teamcode.opmodes.auto;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.util.BulkReading;

@Config
public abstract class AutoBase extends LinearOpMode {

    protected Gamepad currentGamepad = new Gamepad();
    protected Gamepad previousGamepad = new Gamepad();

    protected Pose2d specimenStart;
    protected Pose2d sampleStart;

    protected double specimenStartX = 8, specimenStartY = -63, specimenStartHeading = Math.toRadians(270);

    protected JVBoysSoccerRobot robot;
    protected ArmLift armLift;
    protected ClawSystem clawSystem;

    public static int DEPOSIT_SPECIMEN_FIRST = 3400;
    public static int DEPOSIT_SPECIMEN_POS = DEPOSIT_SPECIMEN_FIRST;
    public static int DEPOSIT_SPECIMEN_SECOND = 3415;
    public static int DEPOSIT_SPECIMEN_DOWN = 2900;
    public static int INTAKE_SPECIMEN_POS = 280;
    public static int INTAKE_SPECIMEN_DOWN = 280;
    public static int DEPOSIT_SAMPLE_POS = Arm.armPreset1DepositSample;
    public static int INTAKE_SAMPLE_POS = Arm.armPresetIntakeSample;
    public static double PIVOT_INTAKE_POS = 0.657;

    protected boolean isBlue = false;

    public void initialize() {
        DEPOSIT_SPECIMEN_POS = DEPOSIT_SPECIMEN_FIRST;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry, true);

        specimenStart = new Pose2d(specimenStartX, specimenStartY, specimenStartHeading);
        sampleStart = new Pose2d(-24, -63, Math.toRadians(270));
        armLift = new ArmLift();
        clawSystem = new ClawSystem();

        PoseStorage.ORIGINAL_INIT_YAW = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        PoseStorage.AUTO_SHIFTED = true;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();
    }

    public class ArmLift {
        private boolean stopUpdate = false;

        public class ExtendSlide implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.slideSubsystem.slideState = LinearSlide.SlideState.BASIC_PID;
                robot.slideSubsystem.referencePos = LinearSlide.slideMaxExtension;
                return false;
            }
        }
        public Action extendSlide() {
            return new ExtendSlide();
        }

        public class DeExtendSlide implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.slideSubsystem.slideState = LinearSlide.SlideState.BASIC_PID;
                robot.slideSubsystem.referencePos = LinearSlide.slideDeExtension;
                return false;
            }
        }
        public Action deExtendSlide() { return new DeExtendSlide(); }

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

        public class DepositSpecimen implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    robot.armSubsystem.setMotionProfile(DEPOSIT_SPECIMEN_POS);
                    robot.armSubsystem.pivotCounter = 3;
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
                    robot.armSubsystem.setIntakeSample(true);
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
    }

    public class ClawSystem {
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.clawSubsystem.closeBothClaw();
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
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
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;

}
