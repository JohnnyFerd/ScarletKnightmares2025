package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;

@Config
public class LinkagePIDController {

    public static double p = 0, i = 0, d = 0, f = 0;
    private final double motorEncoderTicks = 1140;
    private double input = 0, output = 0;
    private double integralSum = 0, lastError = 0;
    private double previousTime = 0;

    private InterpLUT pCoefficients, iCoefficients, dCoefficients;

    private double p_top = 0, i_top = 0, d_top = 0;
    private double p_horizontal = 0, i_horizontal = 0, d_horizontal = 0;

    private final double VERTICAL_POS = 2650;

    public LinkagePIDController() {
        pCoefficients = new InterpLUT();
        iCoefficients = new InterpLUT();
        dCoefficients = new InterpLUT();

        pCoefficients.add(2650, p_top);
        pCoefficients.add(602, p_horizontal);
        pCoefficients.add(4698, p_horizontal);

        iCoefficients.add(2650, i_top);
        iCoefficients.add(602, i_horizontal);
        iCoefficients.add(4698, i_horizontal);

        dCoefficients.add(2650, d_top);
        dCoefficients.add(602, d_horizontal);
        dCoefficients.add(4698, d_horizontal);

        pCoefficients.createLUT();
        iCoefficients.createLUT();
        dCoefficients.createLUT();
    }
    public LinkagePIDController(double p, double i, double d) {
        this();
        this.p = p;
        this.i = i;
        this.d = d;
        f = 0;
    }
    public LinkagePIDController(double p, double i, double d, double f) {
        this();
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

        double output = 0;
        output = (error * p) + (derivative * d) + (integralSum * i);
        return output;
    }

    public double calculatePID(double reference, double state, int armPosition) {
        double p_gs = 0, i_gs = 0, d_gs = 0;

        if (armPosition < 601) {
            armPosition = 601;
        } else if (armPosition > 4697) {
            armPosition = 4697;
        }

        p_gs = pCoefficients.get(armPosition);
        i_gs = iCoefficients.get(armPosition);
        d_gs = dCoefficients.get(armPosition);

        double currentTime = RobotSettings.SUPER_TIME.seconds();
        double error = reference - state;
        integralSum += error * (currentTime - previousTime);
        double derivative = (error - lastError) / (currentTime - previousTime);
        lastError = error;

        previousTime = RobotSettings.SUPER_TIME.seconds();

        double output = 0;
        output = (error * p_gs) + (derivative * d_gs) + (integralSum * i_gs);
        return output;
    }

    /**
     *
     * @param targetPositionArm is the target position of the ARM
     * @return power output of motor
     * this is *technically* a gain schedule
     */
    public double calculateF(double targetPositionArm) {
        // convert target of 375 to 0 degrees
        double degrees = VERTICAL_POS - targetPositionArm;
        degrees = degrees / motorEncoderTicks * 360.0;

//        telemetry.addData("FF Power", Kg * Math.sin(Math.toRadians(degrees)));

        return f * Math.cos( Math.toRadians(degrees) );
    }

}
