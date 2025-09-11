package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.atan2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;

@TeleOp(name = "Diffy Swerve Test", group = "Testing")
public class DiffySwerveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        ElapsedTime timer = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();

        SwerveModule swerve = new SwerveModule("motor1", "motor2", "motor1", hardwareMap, telemetry, timer);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                double x = currentGamepad1.left_stick_x;
                double y = -currentGamepad1.left_stick_y;

                double rawMag = sqrt(pow(x, 2) + pow(y, 2));
                double speed = rawMag / sqrt(2);

                double heading = toDegrees(atan2(y, x));
                swerve.update(speed, heading);

                telemetry.update();
            }
        }
    }
}
