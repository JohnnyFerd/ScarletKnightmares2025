package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@Disabled
@TeleOp (name = "Arm Pivot Test", group = "Testing")
public class ArmPivotTest extends LinearOpMode {

    private HardwareMap hwMap;
    private JVBoysSoccerRobot robot;

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Gamepad currentGamepad2;
    private Gamepad previousGamepad2;

    public static double PRESET1 = 0;
    public static double PRESET2 = 1;

    private enum TestState {
        PRESET1,
        PRESET2,
        NOTHING
    }

    private TestState testState = TestState.PRESET1;

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
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                telemetry.addLine("CONTROLS: ");
                telemetry.addLine("    DPAD UP: Turn servo preset 1, servo preset 2 ");

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
            case PRESET1:
                telemetry.addData("PRESET 1", PRESET1);
                robot.armSubsystem.setPivot(PRESET1);
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    testState = TestState.PRESET2;
                }
                break;
            case PRESET2:
                telemetry.addData("PRESET 2", PRESET2);
                robot.armSubsystem.setPivot(PRESET2);
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    testState = TestState.PRESET1;
                }
                break;
        }

    }
}
