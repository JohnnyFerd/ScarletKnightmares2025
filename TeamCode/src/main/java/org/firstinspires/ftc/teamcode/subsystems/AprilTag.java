package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

// For Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class AprilTag extends Subsystem {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Telemetry telemetry;

    private List<AprilTagDetection> currentDetections;

    // Public variable to store tag label
    public String tagLabel = "";

    public AprilTag(HardwareMap hwMap, Telemetry telemetry) {
        // Merge DS + Dashboard telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Build the AprilTag processor (draw overlays enabled by default)
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)       // draw XYZ axes
                .setDrawCubeProjection(true) // draw cube outline on tag
                .setDrawTagOutline(true) // draw tag boundary
                .build();

        // Open camera and start vision
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Stream the VisionPortal (with overlays) to Dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    @Override
    public void update() {
        currentDetections = aprilTag.getDetections();

        if (currentDetections != null && !currentDetections.isEmpty()) {
            AprilTagDetection detection = currentDetections.get(0); // first one for now
            switch (detection.id) {
                case 20:
                    tagLabel = "bluegoal";
                    break;
                case 21:
                    tagLabel = "GPP";
                    break;
                case 22:
                    tagLabel = "PGP";
                    break;
                case 23:
                    tagLabel = "PPG";
                    break;
                case 24:
                    tagLabel = "redgoal";
                    break;
                default:
                    tagLabel = "";
                    break;
            }
        }
    }

    @Override
    public void addTelemetry() {
        if (currentDetections != null && !currentDetections.isEmpty()) {
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addData("AprilTag ID", detection.id);
                telemetry.addData("Tag Label", tagLabel);

                if (detection.ftcPose != null) {
                    telemetry.addData("X (in)", "%.1f", detection.ftcPose.x);
                    telemetry.addData("Y (in)", "%.1f", detection.ftcPose.y);
                    telemetry.addData("Yaw (deg)", "%.1f", detection.ftcPose.yaw);
                } else if (detection.rawPose != null) {
                    telemetry.addData("Raw X (m)", "%.3f", detection.rawPose.x);
                    telemetry.addData("Raw Y (m)", "%.3f", detection.rawPose.y);
                    telemetry.addData("Raw Z (m)", "%.3f", detection.rawPose.z);

                } else {
                    telemetry.addLine("No pose data available for Tag ID " + detection.id);
                }
            }
        } else {
            telemetry.addLine("No AprilTags detected");
        }
    }

    public AprilTagDetection getBestDetection() {
        if (currentDetections != null && !currentDetections.isEmpty()) {
            return currentDetections.get(0);
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
        FtcDashboard.getInstance().stopCameraStream();
    }
}
