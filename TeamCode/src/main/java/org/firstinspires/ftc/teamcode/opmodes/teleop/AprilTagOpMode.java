package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTag;

@TeleOp(name="AprilTag Test", group="Test")
public class AprilTagOpMode extends LinearOpMode {
    private AprilTag aprilTagSubsystem;

    @Override
    public void runOpMode() {
        aprilTagSubsystem = new AprilTag(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            aprilTagSubsystem.update();
            aprilTagSubsystem.addTelemetry();
            telemetry.update();
        }

        aprilTagSubsystem.stop();
    }
}
