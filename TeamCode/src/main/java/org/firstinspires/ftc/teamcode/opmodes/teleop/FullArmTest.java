package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.pivotPresetRest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@Config
@TeleOp (name = "Full Arm Test", group = "Testing")
public class FullArmTest extends LinearOpMode {

    private HardwareMap hwMap;
    private JVBoysSoccerRobot robot;
    private ElapsedTime runtime = new ElapsedTime();

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Gamepad currentGamepad2;
    private Gamepad previousGamepad2;

    private enum ArmTestState {
        GOING_TO_REST,
        GOING_TO_REST2,
        REST,
        OFF,
        MOVE_ARM
    }

    private ArmTestState armTestState = ArmTestState.REST;
    public double resetTime = 0;

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
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        robot.armSubsystem.armState = Arm.ArmState.AT_REST;

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                telemetry.addLine("CONTROLS: ");
                telemetry.addLine("    DPAD UP: Turn motors on / off ");
                telemetry.addData("    STATE", armTestState);
                armControls();
                clawControls();

                robot.addTelemetry();
                telemetry.update();
                robot.armSubsystem.update();
                robot.BR.readAll();
            }
        }
    }

    public void armControls() {
        switch (armTestState) {
            case REST:
                telemetry.addLine("MOTORS: OFF");
//                robot.armSubsystem.armState = Arm.ArmState.AT_REST;
                robot.armSubsystem.setPivotRest();
                if (currentGamepad1.x && !previousGamepad1.x) {
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setDepositSample();
                    armTestState = ArmTestState.MOVE_ARM;
                }
                if (currentGamepad1.y && !previousGamepad1.y) {
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setDepositSpecimen();
                    armTestState = ArmTestState.MOVE_ARM;
                }
                if (currentGamepad1.a && !previousGamepad1.a) {
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setIntakeSample();
                    armTestState = ArmTestState.MOVE_ARM;
                }
                if (currentGamepad1.b && !previousGamepad1.b) {
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setIntakeSpecimen();
                    armTestState = ArmTestState.MOVE_ARM;
                }
                break;
            case MOVE_ARM:
                telemetry.addLine("MOTORS: ON");
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    robot.armSubsystem.setRest();
                    armTestState = ArmTestState.GOING_TO_REST;
                }
                if (currentGamepad1.x && !previousGamepad1.x) {
                    robot.armSubsystem.setDepositSample();
                }
                if (currentGamepad1.y && !previousGamepad1.y) {
                    robot.armSubsystem.setDepositSpecimen();
                }
                if (currentGamepad1.a && !previousGamepad1.a) {
                    robot.armSubsystem.setIntakeSample();
                }
                if (currentGamepad1.b && !previousGamepad1.b) {
                    robot.armSubsystem.setIntakeSpecimen();
                }
                break;
            case OFF:
                break;
            case GOING_TO_REST:
                if (!robot.armSubsystem.getMP().isBusy()) {
                    resetTime = runtime.seconds();
                    armTestState = ArmTestState.GOING_TO_REST2;
                }
                break;
            case GOING_TO_REST2:
                if (runtime.seconds() - resetTime > 0.5) {
                    robot.armSubsystem.armState = Arm.ArmState.AT_REST;
                    robot.motorArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motorArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armTestState = ArmTestState.REST;
                }
                break;
        }
    }
    public void clawControls() {

        if ((currentGamepad1.left_bumper && !previousGamepad1.left_bumper) || (currentGamepad1.right_bumper && !previousGamepad1.right_bumper)) {
            robot.clawSubsystem.opened = !robot.clawSubsystem.opened;
        }
        if (robot.clawSubsystem.opened) {
            robot.clawSubsystem.openClaw();
        }else {
            robot.clawSubsystem.closeClaw();
        }

    }
}
