package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTag extends Subsystem {

    private VisionPortal visionPortal;
    private AprilTagProcessor processor;
    private Telemetry telemetry;
    private HardwareMap hwMap;
    private List<AprilTagDetection> currentDetections;
    private Size cameraResolution;

    // === Camera & Alignment Constants ===
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final double TAG_SIZE_IN = 6.5;
    public static final double FOCAL_LENGTH_PX = 827;
    public static final double CAMERA_FOV_DEG = 60.0;
    public static final double CAMERA_LATERAL_OFFSET = 6.5;  // inches
    public static final double CAMERA_YAW_OFFSET_DEG = 0.0;
    public static final double CAMERA_TILT_DEG = 0.0;

    // === Labels ===
    public String goalLabel = "None";
    public String patternLabel = "None";

    public AprilTag(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    public void init() {
        cameraResolution = new Size(CAMERA_WIDTH, CAMERA_HEIGHT);

        // FTC-compatible AprilTag processor
        processor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        // VisionPortal setup
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(cameraResolution)

                .addProcessor(processor)
                .build();

        // Optional Dashboard stream
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        telemetry.addLine("AprilTag camera initialized");
    }

    @Override
    public void update() {
        if (processor == null) return;

        currentDetections = processor.getDetections();

        // Reset labels each loop
        goalLabel = "None";
        patternLabel = "None";

        if (currentDetections != null && !currentDetections.isEmpty()) {
            for (AprilTagDetection detection : currentDetections) {
                switch (detection.id) {
                    case 20:
                        goalLabel = "bluegoal";
                        break;
                    case 21:
                        patternLabel = "GPP";
                        break;
                    case 22:
                        patternLabel = "PGP";
                        break;
                    case 23:
                        patternLabel = "PPG";
                        break;
                    case 24:
                        goalLabel = "redgoal";
                        break;
                }
            }
        }
    }

    @Override
    public void addTelemetry() {
        if (currentDetections != null && !currentDetections.isEmpty()) {
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addData("AprilTag ID", detection.id);
                telemetry.addData("Goal Label", goalLabel);
                telemetry.addData("Pattern Label", patternLabel);
                if (detection.rawPose != null) {
                    telemetry.addData("Raw X (m)", "%.2f", detection.rawPose.x);
                    telemetry.addData("Raw Y (m)", "%.2f", detection.rawPose.y);
                    telemetry.addData("Raw Z (m)", "%.2f", detection.rawPose.z);
                }
                telemetry.addData("Distance (in)", "%.2f", getDistanceInches(detection));
                telemetry.addData("Camera Lateral Offset (in)", CAMERA_LATERAL_OFFSET);
                telemetry.addData("Camera Yaw Offset (deg)", CAMERA_YAW_OFFSET_DEG);
                telemetry.addData("Camera Tilt (deg)", CAMERA_TILT_DEG);
            }
        } else {
            telemetry.addLine("No AprilTags detected");
        }
    }

    public AprilTagDetection getLatestTag() {
        if (currentDetections != null && !currentDetections.isEmpty()) {
            return currentDetections.get(0);
        }
        return null;
    }

    public double getDistanceInches(AprilTagDetection detection) {
        if (detection == null) return -1;

        double pixelWidth = Math.abs(detection.corners[0].x - detection.corners[1].x);
        if (pixelWidth <= 0) return -1;

        return (TAG_SIZE_IN * FOCAL_LENGTH_PX) / pixelWidth;
    }

    public int getImageWidth() {
        return cameraResolution.getWidth();
    }

    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }

    public VisionPortal getCamera() {
        return visionPortal;
    }
}
