package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.BALLBALLBALL;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Ball Distance Test")
public class BallDistanceTest extends LinearOpMode {
    // === Gamepad State ===
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();
    private VisionPortal visionPortal;
    private BALLBALLBALL pipeline;
    private JVBoysSoccerRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        // Create pipeline
        pipeline = new BALLBALLBALL();
        pipeline.targetColor = BALLBALLBALL.Target.GREEN;

        // Build VisionPortal (FIXED)
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))

                                   // Make camera appear on DS and Dashboard
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)   // REQUIRED TO SHOW ON DASHBOARD
                .addProcessor(pipeline)
                .build();
// Optional Dashboard stream
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
        telemetry.addLine("Camera started");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateGamepadStates();
            double dist = pipeline.distanceInches;
            if(currentGamepad1.a && !previousGamepad1.a){
                pipeline.toggleTarget();
            }

            if (pipeline.distanceInches > 0) {
                pipeline.driveTowardBall(robot, pipeline);
            } else {
                robot.drivetrainSubsystem.moveXYR(0, 0, 0);
            }
            if (dist > 0) {


                telemetry.addData("Distance (in)", "%.1f", dist);
                telemetry.addData("Center X", "%.1f", pipeline.detectedX);
                telemetry.addData("Center Y", "%.1f", pipeline.detectedY);
            } else {
                telemetry.addLine("No ball detected");
            }

            telemetry.update();
        }

        // Clean shutdown
        visionPortal.close();
    }
    private void updateGamepadStates() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }
}
