package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@Config
@Disabled
@TeleOp (name = "Claw Test", group = "Testing")
public class ClawTest extends LinearOpMode {

    private JVBoysSoccerRobot robot;

    private Gamepad currentGamepad1, previousGamepad1;

    private enum TestState {
        OPEN,
        CLOSED,
        NOTHING
    }
    private TestState testState = TestState.CLOSED;

    @Override
    public void runOpMode() throws InterruptedException {

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addLine("X = open/close claw");

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                clawControls();
                telemetry.addData("Position", robot.servoClawL.getPosition());

                robot.BR.readAll();
                telemetry.update();
            }
        }
    }

    public void clawControls() {
        switch (testState) {
            case OPEN:
                telemetry.addLine("CLAW OPENED");
                robot.clawSubsystem.openBothClaw();
                if (currentGamepad1.x && !previousGamepad1.x) {
                    testState = TestState.CLOSED;
                }
                break;
            case CLOSED:
                telemetry.addLine("CLAW CLOSED");
                robot.clawSubsystem.closeBothClaw();
                if (currentGamepad1.x && !previousGamepad1.x) {
                    testState = TestState.OPEN;
                }
                break;
            case NOTHING:
                break;
        }
    }

}
