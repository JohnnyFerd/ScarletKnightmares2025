    package org.firstinspires.ftc.teamcode.opmodes.auto;

    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.roadrunner.Action;
    import com.acmerobotics.roadrunner.Pose2d;
    import com.acmerobotics.roadrunner.Vector2d;
    import com.acmerobotics.roadrunner.ftc.Actions;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;
    import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
    import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
    import org.firstinspires.ftc.teamcode.subsystems.Shooter;

    @Config
    @Autonomous(name = "Raj Auto Red", group = "Testing")
    public class rajautored extends AutoBase {

        private MecanumDrive drive;
        private AprilTag aprilTag;
        private Shooter shooter;

        @Override
        public void runOpMode() throws InterruptedException {
            // Initialize subsystems
            ElapsedTime timer = new ElapsedTime();
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            aprilTag = new AprilTag(hardwareMap, telemetry);
            shooter = new Shooter(
                    "shooter1", "shooter2",     // motor names
                    "shooterServo1", "shooterServo2", // servo names
                    "paddle1", "paddle2",       // paddle servos
                    hardwareMap,
                    telemetry,
                    timer
            );

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
                    .strafeTo(new Vector2d(0, 50))
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
            telemetry.addData("Detected smth gonna turn to goal", detectedpatternTag);
            telemetry.addData("Detected smth gonna turn to goal", detectedgoalTag);
            Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                 .turn(Math.toRadians(-27))
                 .build()
                );
            for(int i = 0; i<200 ; i++){
                aprilTag.update();
            }
            detectedgoalTag = aprilTag.goalLabel;
            detectedpatternTag = aprilTag.patternLabel;
            telemetry.addData("this goal detected", detectedgoalTag);
            if(detectedgoalTag.equals("redgoal")) {
                  Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                               .strafeTo(new Vector2d(-15, 0))
                               .build()
                    );
            }
            telemetry.addLine("Spinning up shooter...");
            telemetry.update();
            shooter.setVelocity(.5); // full speed
            sleep(1000);
            shooter.togglePaddle(); // simulate firing
            sleep(100);
            shooter.togglePaddle(); // reset paddle
            shooter.setVelocity(0);



          // Stop vision safely
            shooter.update();
            shooter.stop();

            aprilTag.stop();
        }
    }
