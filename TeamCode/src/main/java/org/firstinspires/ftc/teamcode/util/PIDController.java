package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double p, i, d;
    private double input = 0, output = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double integralSum = 0, lastError = 0;

    public PIDController(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double calculatePID(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * p) + (derivative * d) + (integralSum * i);

        return output;
    }

}
