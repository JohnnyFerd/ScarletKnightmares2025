package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;

@Config
public class ArmPIDController {

    // p = 0.0021, i = 0.000015, d = 0.00015, f = 0;
    public static double p_FG = 0.0027, i_FG = 0.00000024, d_FG = 0.00003, f = 0.00035;
    public static double p_G = 0.0014, i_G = 0.00000025, d_G = 0;
    private final double motorEncoderTicks = RobotSettings.TOTAL_ENCODER_TICKS;
    private double input = 0, output = 0;
    private double integralSum = 0, lastError = 0;
    private double previousTime = 0;

    private final double VERTICAL_POS = 2650;

    public ArmPIDController() { }
    public ArmPIDController(double p, double i, double d) {
        this.p_FG = p;
        this.i_FG = i;
        this.d_FG = d;
        f = 0;
    }
    public ArmPIDController(double p, double i, double d, double f) {
        this.p_FG = p;
        this.i_FG = i;
        this.d_FG = d;
        this.f = f;
    }

    public double calculatePID(double reference, double state, boolean fightingGravity) {
        double currentTime = RobotSettings.SUPER_TIME.seconds();
        double error = reference - state;
        integralSum += error * (currentTime - previousTime);
        double derivative = (error - lastError) / (currentTime - previousTime);
        lastError = error;

        previousTime = RobotSettings.SUPER_TIME.seconds();

        double output = 0;
        if (fightingGravity) {
            output = (error * p_FG) + (derivative * d_FG) + (integralSum * i_FG);
        }else {
            output = (error * p_G) + (derivative * d_G) + (integralSum * i_G);
        }
        return output;
    }

    /**
     *
     * @param targetPosition is the target position
     * @return power output of motor
     */
    public double calculateF(double targetPosition) {
        // convert target of 375 to 0 degrees
        double degrees = VERTICAL_POS - targetPosition;
        degrees = degrees / motorEncoderTicks * 360.0;

//        telemetry.addData("FF Power", Kg * Math.sin(Math.toRadians(degrees)));

        return f * Math.sin( Math.toRadians(degrees) );
    }

}
