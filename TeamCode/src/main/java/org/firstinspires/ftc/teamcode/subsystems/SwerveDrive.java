package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

public class SwerveDrive {
    private final SwerveModule leftPod;
    private final SwerveModule rightPod;

    public SwerveDrive(SwerveModule leftPod, SwerveModule rightPod) {
        this.leftPod = leftPod;
        this.rightPod = rightPod;
    }


    public void drive(double x, double y, double rot) {
        double transX = x;
        double transY = y;

        double leftX = transX;
        double leftY = transY - rot;

        double rightX = transX;
        double rightY = transY + rot;

        // polar conversion
        double leftSpeed = hypot(leftX, leftY);
        double leftHeading = toDegrees(atan2(leftY, leftX));

        double rightSpeed = hypot(rightX, rightY);
        double rightHeading = toDegrees(atan2(rightY, rightX));

        // normalize speeds
        double max = Math.max(leftSpeed, rightSpeed);
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        // update pods
        leftPod.update(leftSpeed, leftHeading);
        rightPod.update(rightSpeed, rightHeading);
    }
}
