package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.BulkReading;

@Config
@TeleOp (name = "Arm Test", group = "Testing")
public class ArmTest extends LinearOpMode {

    private HardwareMap hwMap;
    private JVBoysSoccerRobot robot;
    private ElapsedTime runtime = new ElapsedTime();

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Gamepad currentGamepad2;
    private Gamepad previousGamepad2;

    public static int GOAL_POSITION = 0;

    private enum TestState {
        DROP_POS,
        OFF,
        NOTHING
    }

    private TestState testState = TestState.OFF;

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

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                telemetry.addLine("CONTROLS: ");
                telemetry.addLine("    DPAD UP: Turn motors on / off ");
                telemetry.addData("GOAL POSITION", GOAL_POSITION);
                telemetry.addData("CURRENT POSITION", BulkReading.pMotorArmR);
                armControls();

                robot.addTelemetry();
                telemetry.update();
                robot.armSubsystem.update();
                robot.BR.readAll();
            }
        }
    }

    public void armControls() {

        switch (testState) {
            case NOTHING:
                break;
            case OFF:
                telemetry.addLine("MOTORS: OFF");
                robot.armSubsystem.armState = Arm.ArmState.AT_REST;
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    robot.armSubsystem.armState = Arm.ArmState.BASIC_PID;
                    robot.armSubsystem.referencePos = GOAL_POSITION;
                    testState = TestState.DROP_POS;
                }
                break;
            case DROP_POS:
                telemetry.addLine("MOTORS: ON");
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    testState = TestState.OFF;
                }
                break;
        }

    }
}
