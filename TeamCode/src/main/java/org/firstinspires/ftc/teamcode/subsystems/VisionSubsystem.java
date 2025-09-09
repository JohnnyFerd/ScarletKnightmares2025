package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.easyopencv.*;

public class VisionSubsystem {
    private OpenCvCamera camera;
    private BlobDetectionPipeline pipeline;

    public VisionSubsystem(HardwareMap hardwareMap) {
        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        pipeline = new BlobDetectionPipeline();
        camera.setPipeline(pipeline);

        // Async open camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle error if needed
            }
        });
    }

    /** Returns "purple", "green", or "none" */
    public String getDetectedColor() {
        return pipeline.getDetectedColor();
    }

    /** Returns bounding box (null if none detected) */
    public org.opencv.core.Rect getBoundingBox() {
        return pipeline.getBoundingBox();
    }

    /** Stop streaming when you don't need vision */
    public void stop() {
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }
}
