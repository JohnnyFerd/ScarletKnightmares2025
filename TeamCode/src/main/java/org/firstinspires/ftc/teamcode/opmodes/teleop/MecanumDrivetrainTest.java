package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@TeleOp(name = "Mecanum Drivetrain Test", group = "Testing")
public class MecanumDrivetrainTest extends LinearOpMode {

    private HardwareMap hwMap;
    private Telemetry telem;
    private JVBoysSoccerRobot robot;


    private double previousX = 5, previousY = 5, previousR = 5;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Gamepad currentGamepad2;
    private Gamepad previousGamepad2;

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
                telemetry.addLine("    DPAD DOWN: Reset Init Yaw ");
                telemetry.addLine("    B: Swap from field centric to robot centric ");
                telemetry.addLine("    DPAD RIGHT: Toggle Orthogonal Mode ");
                telemetry.addLine("    TRIGGERS: Slowdown the robot by factor of 3 ");

                // Failsafe field-oriented view
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    robot.drivetrainSubsystem.resetInitYaw();
                }


                drivetrainControls();

                robot.addTelemetry();
                telemetry.update();
//                robot.BR.readAll();
            }
        }
    }

    // MECANUM DRIVE
    public void drivetrainControls() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y * -1;
        double r = gamepad1.right_stick_x;

        if (currentGamepad1.b && !previousGamepad1.b) {
            robot.drivetrainSubsystem.isFieldCentric = !robot.drivetrainSubsystem.isFieldCentric;
        }

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            robot.drivetrainSubsystem.orthogonalMode = !robot.drivetrainSubsystem.orthogonalMode;
        }

        if (currentGamepad1.right_trigger > 0.01 || currentGamepad1.left_trigger > 0.01) {
            x /= 3;
            y /= 3;
            r /= 3;
        }

        if (robot.drivetrainSubsystem.lastAngle != null) {
            telemetry.addData("LAST ANGLE", robot.drivetrainSubsystem.lastAngle.firstAngle);
        }
        telemetry.addData("CURRENT REFERENCE ANGLE", 1);
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("R", r);

        // attempting to save motor calls == faster frequency of command calls
        if ( !(previousX == x && previousY == y && previousR == r) ) {
            robot.drivetrainSubsystem.moveXYR(x, y, r);
        }

        previousX = x;
        previousY = y;
        previousR = r;
    }
}
