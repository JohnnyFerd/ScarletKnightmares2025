package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;

@Config
@TeleOp(name = "OneModuleTest", group = "Testing")
public class OneModuleTest extends LinearOpMode {
    public static double Kp = .01;
    public static double Ki = 0;
    public static double Kd = 0;

    public static double heading = 0;
    public static double speed = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        ElapsedTime timer = new ElapsedTime();
        

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();

        SwerveModule swerve = new SwerveModule("pod", "motor1", true, "motor2", true,"motor1", false, hardwareMap, telemetry, timer);

        swerve.toggleTelem(true);
        waitForStart();
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            double x = currentGamepad1.left_stick_x;
            double y = -currentGamepad1.left_stick_y;

            //double speed = sqrt(pow(x, 2) + pow(y, 2));

            //if(x != 0 && y != 0) {heading = toDegrees(atan2(y, x));}

            if (currentGamepad1.a && !previousGamepad1.a) {swerve.toggleKillPow();}

            swerve.update(speed, heading);

            swerve.setPID(Kp, Ki, Kd);

            telemetry.addData("x", x);
            telemetry.addData("y", x);

            telemetry.update();
            }
        }
    }