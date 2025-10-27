package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class SwerveDrive extends Subsystem{
    private final SwerveModule leftPod;
    private final SwerveModule rightPod;

    private Telemetry telemetry;

    public static boolean telemActive = true;
    private ElapsedTime timer;

    private double leftHeading = 0;
    private double rightHeading = 0;

    private boolean killPow = true;

    public SwerveDrive(SwerveModule leftPod, SwerveModule rightPod, Telemetry telemetry, ElapsedTime timer) {
        this.leftPod = leftPod;
        this.rightPod = rightPod;
        this.telemetry = telemetry;
        this.timer = timer;
    }


    public void drive(double x, double y, double rot) {
        double transX = x;
        double transY = y;

        double leftX = transX;
        double leftY = transY - rot;

        double rightX = transX;
        double rightY = transY + rot;


        double leftSpeed = hypot(leftX, leftY);
        double rightSpeed = hypot(rightX, rightY);
        if(x != 0 && y!= 0) {
            leftHeading = toDegrees(atan2(leftY, leftX));
            rightHeading = toDegrees(atan2(rightY, rightX));
        }

        // normalize speeds
        double max = Math.max(leftSpeed, rightSpeed);
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        // update pods
        leftPod.update(leftSpeed, leftHeading);
        rightPod.update(rightSpeed, rightHeading);


        if (telemActive) {
            telemetry.addData("Elapsed time", timer.toString());
            telemetry.addData("Active", !killPow);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("r", rot);
            telemetry.addData("Left Heading", leftPod.getTargetHeading());
            telemetry.addData("Right Heading", rightPod.getTargetHeading());
        }
    }

    public void toggleKillPow()
    {
        killPow = !killPow;
        leftPod.toggleKillPow();
        rightPod.toggleKillPow();
    }

    @Override
    public void addTelemetry() {

    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

    }
}
