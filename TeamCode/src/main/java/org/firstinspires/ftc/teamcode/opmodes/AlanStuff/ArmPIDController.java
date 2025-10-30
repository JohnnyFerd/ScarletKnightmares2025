//package org.firstinspires.ftc.teamcode.opmodes.AlanStuff;
//
//import com.acmerobotics.dashboard.config.Config;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.settings.RobotSettings;
//import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
//import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
//import org.firstinspires.ftc.teamcode.util.BulkReading;
//
//@Config
//public class ArmPIDController {
//
//    private JVBoysSoccerRobot robot;
//    private Telemetry telemetry;
//    // public static double p_BIG = 0.0028, i_BIG = 0.000002, d_BIG = 0.00000000022, f = 0.00035;
//    public static double p_BIG = 0.0025, i_BIG = 0.000003, d_BIG = 0.00000000025, f = 0.0004;
//    public static double p_SMALL = 0, i_SMALL = 0, d_SMALL = 0;
//
//    private final double motorEncoderTicks = RobotSettings.TOTAL_ENCODER_TICKS;
//    private double integralSum = 0, lastError = 0;
//    private double previousTime = 0;
//
//    private double a = 0.8;
//    private double previousFilterEstimate = 0;
//    private double currentFilterEstimate = 0;
//
//    private double previousRefPos = 100000;
//    private double distance = 0;
//
//    private final double VERTICAL_POS = 2780;
//
//
//    public ArmPIDController(JVBoysSoccerRobot robot, Telemetry telemetry) {
//        this.robot = robot;
//        this.telemetry = telemetry;
//        distance = Math.abs(Arm.referencePos - BulkReading.pMotorArmR);
//    }
//    public double calculatePIDSmall(double reference, double state) {
//        return calculatePID(reference, state, p_SMALL, i_SMALL, d_SMALL);
//    }
//    public double calculatePIDBig(double reference, double state) {
//        return calculatePID(reference, state, p_BIG, i_BIG, d_BIG);
//    }
//    public double calculatePID(double reference, double state, double p, double i, double d) {
//        if (previousRefPos != reference) {
//            integralSum = 0;
//        }
//        double currentTime = RobotSettings.SUPER_TIME.seconds();
//        double error = reference - state;
//        integralSum += error * (currentTime - previousTime);
//        double derivative = (error - lastError) / (currentTime - previousTime);
//
//        // LOW PASS FILTER
//        // filter out hight frequency noise to increase derivative performance
//        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * (error-lastError);
//        previousFilterEstimate = currentFilterEstimate;
//
//        // rate of change of the error
//        derivative = currentFilterEstimate / (currentTime - previousTime);
//
//
//        lastError = error;
//
//        previousTime = RobotSettings.SUPER_TIME.seconds();
//        double output;
//        /** if (UseTelemetry.ARM_TELEMETRY) {
//            telemetry.addData("Arm P Value", p);
//            telemetry.addData("Arm I Value", i);
//            telemetry.addData("Arm D Value", d);
//        } **/
//        output = (error * p) + (derivative * d) + (integralSum * i);
//        previousRefPos = reference;
//        return output;
//    }
//
//    /**
//     *
//     * @param targetPosition is the target position
//     * @return power output of motor
//     */
//    public double calculateF(double targetPosition) {
//        // convert target of 375 to 0 degrees
//        double degrees = VERTICAL_POS - targetPosition;
//        degrees = degrees / motorEncoderTicks * 360.0;
//
////        telemetry.addData("FF Power", Kg * Math.sin(Math.toRadians(degrees)));
//
//        return f * Math.sin( Math.toRadians(degrees) );
//    }
//
//}
