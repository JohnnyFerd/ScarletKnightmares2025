package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static java.lang.Math.toDegrees;
import static java.lang.Math.atan2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;

@TeleOp(name = "Diffy Swerve Test", group = "Testing")
public class DiffySwerveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        ElapsedTime timer = new ElapsedTime();
        double heading = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();

        SwerveModule leftPod = new SwerveModule("motor1", true,"motor2",true, "motor1", hardwareMap, telemetry, timer);
        SwerveModule rightPod = new SwerveModule("motor3", true,"motor4",true,"motor3", hardwareMap, telemetry, timer);

        SwerveDrive swerveDrive = new SwerveDrive(leftPod, rightPod, telemetry, timer);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                double x = currentGamepad1.left_stick_x;
                double y = -currentGamepad1.left_stick_y;
                double r = currentGamepad1.right_stick_x;

                if (currentGamepad1.a && !previousGamepad1.a) {swerveDrive.toggleKillPow();}

                swerveDrive.drive(x,y,r);
                telemetry.update();
            }
        }
    }
}
