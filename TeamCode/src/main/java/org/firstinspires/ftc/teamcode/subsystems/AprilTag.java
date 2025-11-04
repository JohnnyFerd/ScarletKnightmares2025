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
    private AprilTagProcessor aprilTag;
    private Telemetry telemetry;

    private List<AprilTagDetection> currentDetections;

    // Camera resolution constants
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    // Tag size in inches
    private static final double TAG_SIZE_IN = 6.5;

    // Focal length in pixels (calibrated)
    private static final double FOCAL_LENGTH_PX = 827;

    // Holds goal label if found
    public String goalLabel = "None";
    public String patternLabel = "None";

    public AprilTag(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;


        // Build the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Open camera and start vision with locked resolution
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(aprilTag)
                .enableLiveView(true)  // show on FTC Dashboard
                .build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    @Override
    public void update() {
        currentDetections = aprilTag.getDetections();

        if (currentDetections != null && !currentDetections.isEmpty()) {
            AprilTagDetection detection = currentDetections.get(0);

            // Assign label based on ID
            switch (detection.id) {
                case 20: goalLabel = "bluegoal"; break;
                case 21: patternLabel = "GPP"; break;
                case 22: patternLabel = "PGP"; break;
                case 23: patternLabel = "PPG"; break;
                case 24: goalLabel = "redgoal"; break;
                default: goalLabel = "None"; break;
            }
        } else {
            goalLabel = "None";
        }
    }

    @Override
    public void addTelemetry() {
        if (currentDetections != null && !currentDetections.isEmpty()) {
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addData("AprilTag ID", detection.id);
                telemetry.addData("Label", goalLabel);
                telemetry.addData("Label", patternLabel);
                // Show raw pose if available
                if (detection.rawPose != null) {
                    telemetry.addData("Raw X (m)", "%.2f", detection.rawPose.x);
                    telemetry.addData("Raw Y (m)", "%.2f", detection.rawPose.y);
                    telemetry.addData("Raw Z (m)", "%.2f", detection.rawPose.z);
                }

                // Show distance estimate
                double distance = calculateDistance(detection);
                telemetry.addData("Distance (in)", "%.2f", distance);
            }
        } else {
            telemetry.addLine("No AprilTags detected");
        }
    }

    /** Returns the first detection, or null if none */
    public AprilTagDetection getLatestTag() {
        if (currentDetections != null && !currentDetections.isEmpty()) {
            return currentDetections.get(0);
        }
        return null;
    }

    /** Calculates distance in inches for a given detection */
    public double getDistanceInches(AprilTagDetection detection) {
        return calculateDistance(detection);
    }

    /** Returns the image width in pixels (needed for centering) */
    public int getImageWidth() {
        return CAMERA_WIDTH;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // Distance calculation using pinhole camera model
    private double calculateDistance(AprilTagDetection detection) {
        if (currentDetections == null) return -1;

        double pixelWidth = Math.abs(detection.corners[0].x - detection.corners[1].x);
        if (pixelWidth <= 0) return -1;

        return (TAG_SIZE_IN * FOCAL_LENGTH_PX) / pixelWidth;
    }

    public VisionPortal getCamera() {
        return visionPortal;
    }
}
