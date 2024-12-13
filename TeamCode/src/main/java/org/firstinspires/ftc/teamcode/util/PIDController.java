package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;

@Config
public class PIDController {

    // p = 0.0021, i = 0.000015, d = 0.00015, f = 0;
    public static double p = 0, i = 0, d = 0, f = 0;
    private final double motorEncoderTicks = RobotSettings.TOTAL_ENCODER_TICKS;
    private double input = 0, output = 0;
    private double integralSum = 0, lastError = 0;
    private double previousTime = 0;

    public PIDController() { }
    public PIDController(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
        f = 0;
    }
    public PIDController(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public double calculatePID(double reference, double state) {
        double currentTime = RobotSettings.SUPER_TIME.seconds();
        double error = reference - state;
        integralSum += error * (currentTime - previousTime);
        double derivative = (error - lastError) / (currentTime - previousTime);
        lastError = error;

        previousTime = RobotSettings.SUPER_TIME.seconds();

        double output = (error * p) + (derivative * d) + (integralSum * i);
        return output;
    }

    /**
     *
     * @param targetPosition is the target position
     * @return power output of motor
     */
    public double calculateF(double targetPosition) {
        // convert target of 375 to 0 degrees
        double degrees = (375.0/1120.0 * 1440.0) - targetPosition;
        degrees = degrees / motorEncoderTicks * 360.0;

//        telemetry.addData("FF Power", Kg * Math.sin(Math.toRadians(degrees)));

        return f * Math.sin( Math.toRadians(degrees) );
    }

}
