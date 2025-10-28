    package org.firstinspires.ftc.teamcode.opmodes.auto;

    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.roadrunner.Action;
    import com.acmerobotics.roadrunner.Pose2d;
    import com.acmerobotics.roadrunner.Vector2d;
    import com.acmerobotics.roadrunner.ftc.Actions;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

    import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;
    import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
    import org.firstinspires.ftc.teamcode.subsystems.AprilTag;

    @Config
    @Autonomous(name = "Raj Simple Auto", group = "Testing")
    public class rajauto extends AutoBase {

        private MecanumDrive drive;
        private AprilTag aprilTag;


        @Override
        public void runOpMode() throws InterruptedException {
            // Initialize subsystems
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            aprilTag = new AprilTag(hardwareMap, telemetry);

            telemetry.addLine("Initializing camera...");
            telemetry.update();

            // Pre-start detection loop
            while (!isStarted() && !isStopRequested()) {
                aprilTag.update();
                aprilTag.addTelemetry();
                telemetry.update();
            }

            waitForStart();
            if (isStopRequested()) {
                aprilTag.stop();
                return;
            }



            // Base trajectory — simple strafe left
            Action moveLeft = drive.actionBuilder(new Pose2d(0, 0, 0))
                    .strafeTo(new Vector2d(0, -50))
                    .build();

            Actions.runBlocking(moveLeft);
           // Once started, read detected tag
            for(int i = 0; i<20 ; i++)
                aprilTag.update();

            String detectedgoalTag = aprilTag.goalLabel;
            String detectedpatternTag = aprilTag.patternLabel;
            telemetry.addData("Detected Tag", detectedgoalTag);
            telemetry.addData("Detected Tag", detectedpatternTag);
            telemetry.update();
            // Optional tag-based adjustment
            if (detectedpatternTag.equals("PPG")) {
                telemetry.addLine("Detected ppg — moving farther forward");
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(-0, 0))
                                .build()
                );
            } else if (detectedpatternTag.equals("PGP")) {
                telemetry.addLine("Detected PGP — moving right slightly");
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                        .turn(Math.toRadians(27))
                                .build()
                );
                for(int i = 0; i<200 ; i++)
                    aprilTag.update();

                detectedgoalTag = aprilTag.goalLabel;
                detectedpatternTag = aprilTag.patternLabel;
                telemetry.addLine("bluegoal detected");
                if(detectedgoalTag.equals("bluegoal")) {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(new Vector2d(-15, 0))
                                    .build()
                    );
                }
            } else {
               telemetry.addLine("No tag — default path");
            }

            telemetry.update();

          // Stop vision safely
           aprilTag.stop();
        }
    }
