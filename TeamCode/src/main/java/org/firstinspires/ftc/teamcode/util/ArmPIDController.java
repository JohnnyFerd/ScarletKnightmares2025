package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@Config
public class ArmPIDController {

    private JVBoysSoccerRobot robot;
    public static double FG_p = 0.0032, FG_i = 0.00000076, FG_d = 0.000025, f = 0.055;
    public static double G_p = 0.0017, G_i = 0.0000013, G_d = 0.000009;

    // Distance of 0-150 ticks
    public static double FG_p100 = 0, FG_i100 = 0, FG_d100 = 0;
    public static double G_p100 = 0, G_i100 = 0, G_d100 = 0;

    // Distance of 150-350 ticks
    public static double FG_p400 = 0, FG_i400 = 0, FG_d400 = 0;
    public static double G_p400 = 0, G_i400 = 0, G_d400 = 0;

    // TODO: fine tune these threshold values for changing pid values for most accuracy
    public static int THRESHOLD1 = 150;
    public static int THRESHOLD2 = 400;

    private final double motorEncoderTicks = RobotSettings.TOTAL_ENCODER_TICKS;
    private double integralSum = 0, lastError = 0;
    private double previousTime = 0;

    private double previousRefPos = 100000;

    private final double VERTICAL_POS = 2750;

    public ArmPIDController(JVBoysSoccerRobot robot) {
        this.robot = robot;
    }

    private double distance = Math.abs(robot.armSubsystem.referencePos - BulkReading.pMotorArmR);
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
        if (!fightingGravity) {
            if (distance <= THRESHOLD1) {
                p = G_p100;
                i = G_i100;
                d = G_d100;
            }else if (distance <= THRESHOLD2) {
                p = G_p400;
                i = G_i400;
                d = G_d400;
            }else {
                p = G_p;
                i = G_i;
                d = G_d;
            }
        }else {
            if (distance <= THRESHOLD1) {
                p = FG_p100;
                i = FG_i100;
                d = FG_d100;
            }else if (distance <= THRESHOLD2) {
                p = FG_p400;
                i = FG_i400;
                d = FG_d400;
            }else {
                p = FG_p;
                i = FG_i;
                d = FG_d;
            }
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
