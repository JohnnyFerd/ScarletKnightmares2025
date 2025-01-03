package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@Config
public class ArmPIDController {

    private JVBoysSoccerRobot robot;
    private Telemetry telemetry;
    public static double FG_p = 0.0028, FG_i = 0.0000028, FG_d = 0.000000002, f = 0.004;
    public static double G_p = 0.0028, G_i = 0.0000028, G_d = 0;
    //  public static double G_p = 0.0032, G_i = 0.00000076, G_d = 0.000025;

    private final double motorEncoderTicks = RobotSettings.TOTAL_ENCODER_TICKS;
    private double integralSum = 0, lastError = 0;
    private double previousTime = 0;

    private double previousRefPos = 100000;
    private double distance = 0;

    private final double VERTICAL_POS = 2750;


    public ArmPIDController(JVBoysSoccerRobot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        distance = Math.abs(Arm.referencePos - BulkReading.pMotorArmR);
    }
    public double calculatePID(double reference, double state, boolean fightingGravity) {
        double p;
        double i;
        double d;

        double currentTime = RobotSettings.SUPER_TIME.seconds();
        double error = reference - state;
        integralSum += error * (currentTime - previousTime);
        double derivative = (error - lastError) / (currentTime - previousTime);
        lastError = error;

        previousTime = RobotSettings.SUPER_TIME.seconds();

        double output;

        // TODO: Check if this works or not
        if (previousRefPos != reference) {
            distance = Math.abs(state - reference);
        }
//        distance = Math.abs(previousRefPos - reference);
//        if (!fightingGravity) {
//            p = G_p;
//            i = G_i;
//            d = G_d;
//        }else {
            p = FG_p;
            i = FG_i;
            d = FG_d;
//        }

        if (UseTelemetry.ARM_TELEMETRY) {
            telemetry.addData("Arm P Value", p);
            telemetry.addData("Arm I Value", i);
            telemetry.addData("Arm D Value", d);
        }

        output = (error * p) + (derivative * d) + (integralSum * i);
        previousRefPos = reference;
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
