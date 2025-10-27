package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "ShooterTester", group = "Testing")
public class ShooterTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        ElapsedTime timer = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        Shooter shooter = new Shooter("shooter1", "shooter2", "1shooter", "2shooter", "paddle1", "paddle2", hardwareMap, telemetry, timer);
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (currentGamepad1.a && !previousGamepad1.a) {shooter.togglePaddle();}
                shooter.setVelocity(currentGamepad1.right_trigger);
            }
        }
    }
}