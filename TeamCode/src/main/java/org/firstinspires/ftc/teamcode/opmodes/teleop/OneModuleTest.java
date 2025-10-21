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


    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        ElapsedTime timer = new ElapsedTime();
        double heading = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();

        SwerveModule swerve = new SwerveModule("pod", "motor1", true, "motor2", true,"motor1", hardwareMap, telemetry, timer);

        swerve.toggleTelem(true);
        waitForStart();
        if (opModeIsActive()) {

                double x = currentGamepad1.left_stick_x;
                double y = -currentGamepad1.left_stick_y;

                double rawMag = sqrt(pow(x, 2) + pow(y, 2));
                double speed = rawMag / sqrt(2);        //making sure speed <= 1

                if(x != 0 && y != 0) {heading = toDegrees(atan2(y, x));}

                if (currentGamepad1.a && !previousGamepad1.a) {swerve.toggleKillPow();}

                swerve.update(speed, heading);

                telemetry.update();
            }
        }
    }