package org.firstinspires.ftc.teamcode.subsystems ;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class BALLBALLBALL implements VisionProcessor {

    public enum Target { GREEN, PURPLE }
    public Target targetColor = Target.GREEN;

    // Output data
    public double detectedX = -1;
    public double detectedY = -1;
    public double distanceInches = -1;
    private Rect detectedRect = null;

    private Mat mat = new Mat();
    private Mat hsv = new Mat();
    private Mat mask = new Mat();

    private final Paint boxPaint;

    public BALLBALLBALL() {
        boxPaint = new Paint();
        boxPaint.setColor(Color.GREEN);
        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setStrokeWidth(5);
    }
    public void driveTowardBall(JVBoysSoccerRobot robot, BALLBALLBALL pipe) {

        if (pipe.distanceInches <= 0) {
            robot.drivetrainSubsystem.moveXYR(0, 0, 0);
            return;
        }

        double x = pipe.detectedX;
        double frameCenter = 640 / 2.0;   // camera width
        double error = (x - frameCenter); // positive if ball is to the right

        // Normalize into [-1, 1]
        double turnCmd = error / frameCenter;

        // Gain (tune)
        turnCmd *= 0.6;

        // Forward power depends on distance
        double forward = 0.25;
        if (pipe.distanceInches < 10) {
            forward = 0; // stop when very close
        }

        robot.drivetrainSubsystem.moveXYR(forward, 0, -turnCmd);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    public void toggleTarget() {
        if (targetColor == Target.GREEN) {
            targetColor = Target.PURPLE;
            boxPaint.setColor(Color.MAGENTA);
        } else {
            targetColor = Target.GREEN;
            boxPaint.setColor(Color.GREEN);
        }
    }

    @Override
    public Object processFrame(Mat frame, long timestamp) {

        // Convert to HSV
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        // Color thresholds
        Scalar low, high;

        if (targetColor == Target.GREEN) {
            low = new Scalar(35, 80, 60);
            high = new Scalar(90, 255, 255);
        } else {
            low = new Scalar(125, 60, 50);
            high = new Scalar(165, 255, 255);
        }

        // Mask
        Core.inRange(hsv, low, high, mask);

        // Morph cleanup
        Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        detectedRect = null;
        double maxArea = 0;

        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area < 200) continue;

            Rect r = Imgproc.boundingRect(c);

            if (area > maxArea) {
                maxArea = area;
                detectedRect = r;
            }
        }

        if (detectedRect != null) {
            detectedX = detectedRect.x + detectedRect.width / 2.0;
            detectedY = detectedRect.y + detectedRect.height / 2.0;

            double pixelWidth = detectedRect.width;

            // Distance estimation model
            distanceInches = 3892.6829268 / pixelWidth;
        } else {
            detectedX = -1;
            detectedY = -1;
            distanceInches = -1;
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        if (detectedRect != null) {
            float left = detectedRect.x * scaleBmpPxToCanvasPx;
            float top = detectedRect.y * scaleBmpPxToCanvasPx;
            float right = (detectedRect.x + detectedRect.width) * scaleBmpPxToCanvasPx;
            float bottom = (detectedRect.y + detectedRect.height) * scaleBmpPxToCanvasPx;

            canvas.drawRect(left, top, right, bottom, boxPaint);

            Paint text = new Paint();
            text.setColor(Color.WHITE);
            text.setTextSize(40 * scaleCanvasDensity);

            canvas.drawText(String.format("%.1f in", distanceInches),
                    left, top - 10, text);
        }
    }
}
