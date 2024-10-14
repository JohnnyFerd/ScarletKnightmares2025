package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Rigging;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;

@Config
@TeleOp (name = "Rigging Test", group = "Testing")
public class RiggingTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private double rigWaitTime = 0;

    private Gamepad currentGamepad1, previousGamepad1;

    private enum RiggingControlsState {
        DOWN_WAIT,
        DOWN,
        UP,
        HANGING,
        NOTHING
    }
    private RiggingControlsState hangState = RiggingControlsState.DOWN;

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

                telemetry.addLine("DPAD BUTTONS = undo string");
                telemetry.addLine("No motor code yet");

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                riggingControls();

                if (currentGamepad1.dpad_left) {
                    robot.motorRigL.setPower(-1.0 * RobotSettings.RIGGING_MOTOR_SPEED * 0.5);
                }else {
                    robot.motorRigL.setPower(0);
                }

                if (currentGamepad1.dpad_right) {
                    robot.motorRigR.setPower(-1.0 * RobotSettings.RIGGING_MOTOR_SPEED * 0.5);
                }else {
                    robot.motorRigR.setPower(0);
                }

                if (currentGamepad1.dpad_up) {
                    robot.motorRigL.setPower(1.0 * RobotSettings.RIGGING_MOTOR_SPEED * 0.5);
                }else {
                    robot.motorRigL.setPower(0);
                }

                if (currentGamepad1.dpad_down) {
                    robot.motorRigR.setPower(1.0 * RobotSettings.RIGGING_MOTOR_SPEED * 0.5);
                }else {
                    robot.motorRigR.setPower(0);
                }

                robot.riggingSubsystem.addTelemetry();
                robot.riggingSubsystem.update();
                robot.BR.readAll();
                telemetry.update();
            }
        }
    }

    public void riggingControls() {
        switch (hangState) {
            case UP:
                robot.riggingSubsystem.riggingState = Rigging.RiggingState.ARMS_UP;
                if (currentGamepad1.left_bumper || currentGamepad1.right_bumper) {
                    hangState = RiggingControlsState.HANGING;
                }
                if (currentGamepad1.x && !previousGamepad1.x) {
                    rigWaitTime = runtime.seconds();
                    hangState = RiggingControlsState.DOWN_WAIT;
                }
                break;
            case DOWN:
                robot.riggingSubsystem.riggingState = Rigging.RiggingState.SERVOS_OFF;
                if (currentGamepad1.x && !previousGamepad1.x) {
                    hangState = RiggingControlsState.UP;
                }
                break;
            case DOWN_WAIT:
                robot.riggingSubsystem.riggingState = Rigging.RiggingState.ARMS_RESTING;
                if (runtime.seconds() - rigWaitTime > 1.0) {
                    hangState = RiggingControlsState.DOWN;
                }
                break;
            case HANGING:
                robot.riggingSubsystem.riggingState = Rigging.RiggingState.SERVOS_OFF;
                if (currentGamepad1.left_bumper || currentGamepad1.right_bumper) {
                    robot.motorRigL.setPower(RobotSettings.RIGGING_MOTOR_SPEED);
                    robot.motorRigR.setPower(RobotSettings.RIGGING_MOTOR_SPEED);
                }else {
                    robot.motorRigL.setPower(0);
                    robot.motorRigR.setPower(0);
                }
                if (currentGamepad1.x && !previousGamepad1.x) {
                    rigWaitTime = runtime.seconds();
                    hangState = RiggingControlsState.DOWN_WAIT;
                }
                break;
            case NOTHING:
                break;
        }
    }

}
