package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;

@Config
@Autonomous
public class PIDToPoint extends LinearOpMode {

    private PIDCoefficients pidCoefficients;
    private MecanumDrive drive;

    public static Pose2d targetPose = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        RobotSettings.SUPER_TIME.reset();

        drive = new MecanumDrive(hardwareMap, targetPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                goToPoint(targetPose);

                telemetry.update();
            }
        }
    }

    public void goToPoint(Pose2d target) {
//        drive.();
    }
}
