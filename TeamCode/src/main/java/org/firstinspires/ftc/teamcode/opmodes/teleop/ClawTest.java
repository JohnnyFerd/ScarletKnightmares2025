package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Rigging;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;

@Config
@TeleOp (name = "Claw Test", group = "Testing")
public class ClawTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;

    private Gamepad currentGamepad1, previousGamepad1;

    private enum ClawControlsState {
        OPEN,
        CLOSED,
        NOTHING
    }
    private ClawControlsState clawState = ClawControlsState.CLOSED;

    @Override
    public void runOpMode() throws InterruptedException {

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addLine("X = open/close claw");

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                clawControls();

                robot.clawSubsystem.addTelemetry();
                robot.clawSubsystem.update();
                robot.BR.readAll();
                telemetry.update();
            }
        }
    }

    public void clawControls() {
        switch (clawState) {
            case OPEN:
                robot.clawSubsystem.openClaw();
                if (currentGamepad1.x && !previousGamepad1.x) {
                    clawState = ClawControlsState.CLOSED;
                }
                break;
            case CLOSED:
                robot.clawSubsystem.closeClaw();
                if (currentGamepad1.x && !previousGamepad1.x) {
                    clawState = ClawControlsState.OPEN;
                }
                break;
            case NOTHING:
                break;
        }
    }

}
