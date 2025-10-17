package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static java.lang.Math.toDegrees;
import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;
@Config
@TeleOp(name = "Diffy Swerve Test", group = "Testing")
public class DiffySwerveTest extends LinearOpMode {

    public static double kP1 = .1;
    public static double kI1 = 0;
    public static double kD1 = 0;

    public static double kP2 = .1;
    public static double kI2 = 0;
    public static double kD2 = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        ElapsedTime timer = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();

        SwerveModule leftPod = new SwerveModule("leftPod","motor1", true,"motor2",true, "motor1", false, hardwareMap, telemetry, timer);
        SwerveModule rightPod = new SwerveModule("rightPod","motor3", true,"motor4",false,"motor3", false, hardwareMap, telemetry, timer);

        SwerveDrive swerveDrive = new SwerveDrive(leftPod, rightPod, telemetry, timer);
        swerveDrive.toggleModuleTelem(true);

        double x = 0;
        double y = 0;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);


                if (gamepad1.left_stick_y != 0 && gamepad1.left_stick_x != 0) {
                    x = currentGamepad1.left_stick_x;
                    y = -currentGamepad1.left_stick_y;
                }
                double r = currentGamepad1.right_stick_x;

                if (currentGamepad1.a && !previousGamepad1.a) {swerveDrive.toggleKillPow();}

                swerveDrive.drive(x,y,r);
                swerveDrive.setPID(kP1,kI1,kD1,kP2,kI2,kD2);
                telemetry.update();
            }
        }
    }
}
